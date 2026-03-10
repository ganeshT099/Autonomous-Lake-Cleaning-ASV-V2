#!/usr/bin/env python3
"""
ASV Thruster Control Node
Subscribes: /cmd_vel (geometry_msgs/Twist)
Sends PWM commands to ESP32 over Serial
ESP32 → PCA9685 → 2x ESC → 2x T200 Thruster

Differential Drive:
  Forward  : both thrusters forward
  Backward : both thrusters reverse
  Left     : right faster than left
  Right    : left faster than right

PWM Range: 1100us (full reverse) - 1500us (stop) - 1900us (full forward)
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import serial
import time
import threading

class ThrusterNode(Node):
    # PWM limits for T200 + 30A ESC
    PWM_MIN     = 1100   # full reverse
    PWM_NEUTRAL = 1500   # stop
    PWM_MAX     = 1900   # full forward
    PWM_RANGE   = 400    # PWM_MAX - PWM_NEUTRAL

    def __init__(self):
        super().__init__('asv_thruster_node')

        self.declare_parameter('port',        '/dev/ttyUSB0')
        self.declare_parameter('baud_rate',    115200)
        self.declare_parameter('max_linear',   1.0)
        self.declare_parameter('max_angular',  1.0)
        self.declare_parameter('safety_stop',  True)
        self.declare_parameter('cmd_timeout',  1.0)  # stop if no cmd for 1s

        self.port        = self.get_parameter('port').value
        self.baud        = self.get_parameter('baud_rate').value
        self.max_linear  = self.get_parameter('max_linear').value
        self.max_angular = self.get_parameter('max_angular').value
        self.safety_stop = self.get_parameter('safety_stop').value
        self.cmd_timeout = self.get_parameter('cmd_timeout').value

        # Publishers
        self.status_pub = self.create_publisher(String, '/asv/thruster/status', 10)

        # Subscriber
        self.cmd_sub = self.create_subscription(
            Twist, '/cmd_vel', self._cmd_vel_callback, 10)

        # Safety subscriber — stop if DANGER
        self.safety_sub = self.create_subscription(
            String, '/asv/ultrasonic/safety', self._safety_callback, 10)

        # State
        self.last_cmd_time = time.time()
        self.safety_level  = 'SAFE'
        self.left_pwm      = self.PWM_NEUTRAL
        self.right_pwm     = self.PWM_NEUTRAL
        self.lock          = threading.Lock()
        self.ser           = None

        self._connect_serial()

        # Watchdog timer — stops thrusters if no cmd received
        self.watchdog = self.create_timer(0.1, self._watchdog)
        # Publish status at 10Hz
        self.status_timer = self.create_timer(0.1, self._publish_status)

        self.get_logger().info(f'Thruster node started | port={self.port}')
        self.get_logger().info('Waiting for /cmd_vel commands...')

    def _connect_serial(self):
        while True:
            try:
                self.ser = serial.Serial(self.port, self.baud, timeout=1.0)
                self.get_logger().info(f'ESP32 connected on {self.port}')
                # Send neutral on connect
                self._send_pwm(self.PWM_NEUTRAL, self.PWM_NEUTRAL)
                break
            except serial.SerialException as e:
                self.get_logger().error(f'Serial error: {e} retrying in 3s')
                time.sleep(3.0)

    def _vel_to_pwm(self, left_vel, right_vel):
        """Convert velocity (-1.0 to 1.0) to PWM (1100-1900)"""
        left_vel  = max(-1.0, min(1.0, left_vel))
        right_vel = max(-1.0, min(1.0, right_vel))
        left_pwm  = int(self.PWM_NEUTRAL + left_vel  * self.PWM_RANGE)
        right_pwm = int(self.PWM_NEUTRAL + right_vel * self.PWM_RANGE)
        return left_pwm, right_pwm

    def _send_pwm(self, left_pwm, right_pwm):
        """Send PWM command to ESP32
        Format: $THR,L:1500,R:1500*\n
        """
        try:
            cmd = f'$THR,L:{left_pwm},R:{right_pwm}*\n'
            self.ser.write(cmd.encode())
            with self.lock:
                self.left_pwm  = left_pwm
                self.right_pwm = right_pwm
        except serial.SerialException as e:
            self.get_logger().warn(f'Serial send error: {e}')
            self._connect_serial()

    def _cmd_vel_callback(self, msg):
        self.last_cmd_time = time.time()

        # Safety check
        if self.safety_stop and self.safety_level == 'DANGER':
            self.get_logger().warn('DANGER detected — thrusters stopped!')
            self._send_pwm(self.PWM_NEUTRAL, self.PWM_NEUTRAL)
            return

        # Slow down on warning
        speed_limit = 1.0
        if self.safety_level == 'WARNING':
            speed_limit = 0.5
            self.get_logger().warn('WARNING zone — speed limited to 50%')

        # Differential drive mixing
        linear  = msg.linear.x  / self.max_linear  * speed_limit
        angular = msg.angular.z / self.max_angular * speed_limit

        # Mix linear + angular for differential drive
        left_vel  = linear - angular
        right_vel = linear + angular

        # Clamp
        left_vel  = max(-1.0, min(1.0, left_vel))
        right_vel = max(-1.0, min(1.0, right_vel))

        left_pwm, right_pwm = self._vel_to_pwm(left_vel, right_vel)
        self._send_pwm(left_pwm, right_pwm)

    def _safety_callback(self, msg):
        self.safety_level = msg.data
        if msg.data == 'DANGER':
            self._send_pwm(self.PWM_NEUTRAL, self.PWM_NEUTRAL)

    def _watchdog(self):
        """Stop thrusters if no cmd received within timeout"""
        if time.time() - self.last_cmd_time > self.cmd_timeout:
            self._send_pwm(self.PWM_NEUTRAL, self.PWM_NEUTRAL)

    def _publish_status(self):
        with self.lock:
            l = self.left_pwm
            r = self.right_pwm
        s = String()
        s.data = (
            f'[THRUSTER] L={l}us R={r}us | '
            f'safety={self.safety_level}'
        )
        self.status_pub.publish(s)

    def destroy_node(self):
        self._send_pwm(self.PWM_NEUTRAL, self.PWM_NEUTRAL)
        if self.ser and self.ser.is_open:
            self.ser.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = ThrusterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
