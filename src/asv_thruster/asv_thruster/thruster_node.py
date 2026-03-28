#!/usr/bin/env python3
"""
ASV Thruster Node - FINAL PAKKA VERSION
=========================================
Subscribes: /cmd_vel (geometry_msgs/Twist)
Controls:   PCA9685 → ESC → T200 Thrusters

Hardware confirmed:
  PCA9685  → i2c-7, address 0x40
  Left ESC → Channel 0
  Right ESC→ Channel 1

PWM confirmed:
  Neutral  = 1500us
  Forward  = 1800us (max)
  Reverse  = 1250us (max)
  Min spin = 1750us (threshold!)
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import smbus2
import time
import threading
import signal
import sys

# ── Hardware config ───────────────────────────────────────────
I2C_BUS      = 7
PCA_ADDR     = 0x40
LEFT_CH      = 0
RIGHT_CH     = 1

# ── Confirmed PWM values ──────────────────────────────────────
PWM_NEUTRAL  = 1500
PWM_MAX      = 1800
PWM_MIN      = 1250
PWM_MIN_SPIN = 1750   # T200 minimum spin threshold!
PWM_RANGE    = 300

# ── Control config ────────────────────────────────────────────
CMD_TIMEOUT  = 0.5    # Stop if no cmd_vel for 0.5s
LOOP_HZ      = 20     # Control loop frequency


class PCA9685:
    def __init__(self, bus_num=I2C_BUS, address=PCA_ADDR):
        self.bus  = smbus2.SMBus(bus_num)
        self.addr = address
        self._init()
        print('PCA9685 initialized ✅')

    def _init(self):
        # Correct init sequence!
        self.bus.write_byte_data(self.addr, 0x00, 0x10)  # sleep
        time.sleep(0.1)
        self.bus.write_byte_data(self.addr, 0xFE, 0x7A)  # prescaler 50Hz
        time.sleep(0.1)
        self.bus.write_byte_data(self.addr, 0x00, 0x00)  # wake
        time.sleep(0.1)

    def set_pwm(self, channel, us):
        duty = int((us / 20000.0) * 4096)
        reg  = 0x06 + (channel * 4)
        self.bus.write_byte_data(self.addr, reg,     0x00)
        self.bus.write_byte_data(self.addr, reg + 1, 0x00)
        self.bus.write_byte_data(self.addr, reg + 2, duty & 0xFF)
        self.bus.write_byte_data(self.addr, reg + 3, duty >> 8)

    def neutral(self):
        self.set_pwm(LEFT_CH,  PWM_NEUTRAL)
        self.set_pwm(RIGHT_CH, PWM_NEUTRAL)

    def hard_stop(self):
        for _ in range(20):
            self.set_pwm(LEFT_CH,  PWM_NEUTRAL)
            self.set_pwm(RIGHT_CH, PWM_NEUTRAL)
            time.sleep(0.02)


class ThrusterNode(Node):
    def __init__(self):
        super().__init__('asv_thruster_node')

        # ── State ─────────────────────────────────────────────
        self.left_pwm   = PWM_NEUTRAL
        self.right_pwm  = PWM_NEUTRAL
        self.target_left  = PWM_NEUTRAL
        self.target_right = PWM_NEUTRAL
        self.last_cmd   = time.time()
        self.armed      = False
        self.lock       = threading.Lock()

        # ── PCA9685 ───────────────────────────────────────────
        try:
            self.pca = PCA9685()
            self.pca.neutral()
        except Exception as e:
            self.get_logger().error(f'PCA9685 ERROR: {e}')
            self.pca = None

        # ── Arm ESCs in background thread ─────────────────────
        arm_t = threading.Thread(target=self._arm_escs)
        arm_t.daemon = True
        arm_t.start()

        # ── Subscriber ────────────────────────────────────────
        self.create_subscription(
            Twist, '/cmd_vel', self._cmd_cb, 10)

        # ── Publishers ────────────────────────────────────────
        self.status_pub = self.create_publisher(
            String, '/asv/thruster/status', 10)

        # ── Control loop 20Hz ─────────────────────────────────
        self.create_timer(1.0 / LOOP_HZ, self._control_loop)

        # ── Watchdog 10Hz ─────────────────────────────────────
        self.create_timer(0.1, self._watchdog)

        self.get_logger().info('Thruster node started!')
        self.get_logger().info('Waiting for ESC arm...')

    def _arm_escs(self):
        """Arm ESCs in background — does not block ROS2!"""
        if self.pca is None:
            return
        self.get_logger().info('Arming ESCs — 3 seconds...')
        self.pca.set_pwm(LEFT_CH,  PWM_NEUTRAL)
        self.pca.set_pwm(RIGHT_CH, PWM_NEUTRAL)
        time.sleep(3.0)
        self.armed = True
        self.get_logger().info('ESCs armed! ✅')
        self.get_logger().info('Ready for /cmd_vel commands!')

    def _to_pwm(self, value):
        """Convert -1.0 to +1.0 to PWM microseconds"""
        if value > 0.0:
            # Forward: 1500 → 1800
            pwm = PWM_NEUTRAL + int(value * PWM_RANGE)
            # Apply min spin threshold
            if pwm > PWM_NEUTRAL:
                pwm = max(pwm, PWM_MIN_SPIN)
        elif value < 0.0:
            # Reverse: 1500 → 1250
            pwm = PWM_NEUTRAL + int(value * PWM_RANGE)
            # Apply min reverse threshold
            if pwm < PWM_NEUTRAL:
                pwm = min(pwm, PWM_NEUTRAL - (PWM_MIN_SPIN - PWM_NEUTRAL))
        else:
            pwm = PWM_NEUTRAL

        # Final clamp
        return max(PWM_MIN, min(PWM_MAX, pwm))

    def _cmd_cb(self, msg):
        """Receive cmd_vel and convert to target PWM"""
        if not self.armed:
            return

        self.last_cmd = time.time()

        linear  = float(msg.linear.x)
        angular = float(msg.angular.z)

        # Differential mixing
        # left  = linear - angular
        # right = linear + angular
        left_out  = linear - angular
        right_out = linear + angular

        # Clamp to -1.0 to +1.0
        left_out  = max(-1.0, min(1.0, left_out))
        right_out = max(-1.0, min(1.0, right_out))

        with self.lock:
            self.target_left  = self._to_pwm(left_out)
            self.target_right = self._to_pwm(right_out)

    def _watchdog(self):
        """Stop if no cmd_vel received for timeout"""
        if self.armed and time.time() - self.last_cmd > CMD_TIMEOUT:
            with self.lock:
                self.target_left  = PWM_NEUTRAL
                self.target_right = PWM_NEUTRAL

    def _control_loop(self):
        """Smooth ramp to target PWM at 20Hz"""
        if self.pca is None or not self.armed:
            return

        with self.lock:
            target_left  = self.target_left
            target_right = self.target_right

        # Ramp step per loop
        # At 20Hz, step=10 → takes 1.5 sec full range
        # At 20Hz, step=20 → takes 0.75 sec full range
        step = 15

        # Ramp left
        if self.left_pwm < target_left:
            self.left_pwm = min(
                target_left, self.left_pwm + step)
        elif self.left_pwm > target_left:
            self.left_pwm = max(
                target_left, self.left_pwm - step)

        # Ramp right
        if self.right_pwm < target_right:
            self.right_pwm = min(
                target_right, self.right_pwm + step)
        elif self.right_pwm > target_right:
            self.right_pwm = max(
                target_right, self.right_pwm - step)

        # Immediate stop if target is neutral
        if target_left == PWM_NEUTRAL:
            self.left_pwm = PWM_NEUTRAL
        if target_right == PWM_NEUTRAL:
            self.right_pwm = PWM_NEUTRAL

        # Send to PCA9685
        self.pca.set_pwm(LEFT_CH,  self.left_pwm)
        self.pca.set_pwm(RIGHT_CH, self.right_pwm)

        # Publish status
        s = String()
        if not self.armed:
            s.data = '[THR] Arming...'
        elif self.left_pwm == PWM_NEUTRAL and \
             self.right_pwm == PWM_NEUTRAL:
            s.data = '[THR] Stopped'
        else:
            s.data = (f'[THR] L={self.left_pwm}us '
                      f'R={self.right_pwm}us')
        self.status_pub.publish(s)

    def safe_stop(self):
        """Hard stop all thrusters"""
        self.get_logger().info('Stopping thrusters...')
        if self.pca:
            self.pca.hard_stop()
        self.get_logger().info('Thrusters stopped! ✅')


def main():
    rclpy.init()
    node = ThrusterNode()

    def shutdown(sig, frame):
        print('\nShutdown received!')
        node.safe_stop()
        try:
            node.destroy_node()
            rclpy.shutdown()
        except Exception:
            pass
        print('Safe shutdown complete! ✅')
        sys.exit(0)

    signal.signal(signal.SIGINT,  shutdown)
    signal.signal(signal.SIGTERM, shutdown)

    try:
        rclpy.spin(node)
    except SystemExit:
        pass


if __name__ == '__main__':
    main()
