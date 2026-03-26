#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import smbus
import time


class ThrusterNode(Node):

    def __init__(self):
        super().__init__('thruster_node')

        # 🔥 I2C BUS (Jetson correct bus)
        self.bus = smbus.SMBus(7)
        self.addr = 0x40

        # PWM init
        self._init_pwm()

        # ESC limits
        self.NEUTRAL = 1500
        self.MAX = 1850
        self.MIN = 1250

        # Control tuning
        self.scale = 400
        self.turn_gain = 0.6
        self.deadzone = 0.05

        # State
        self.left_pwm = self.NEUTRAL
        self.right_pwm = self.NEUTRAL
        self.last_time = time.time()
        self.timeout = 0.3   # 🔥 faster stop

        # ROS
        self.create_subscription(Twist, '/cmd_vel', self.callback, 10)
        self.create_timer(0.05, self.loop)

        self.get_logger().info("🚤 Thruster Node (FINAL) Started")

        self.arm_esc()

    # 🔧 INIT PCA9685
    def _init_pwm(self):
        bus = self.bus
        addr = self.addr

        bus.write_byte_data(addr, 0x00, 0x00)

        prescale = int(25000000.0 / (4096 * 50) - 1)

        oldmode = bus.read_byte_data(addr, 0x00)
        newmode = (oldmode & 0x7F) | 0x10

        bus.write_byte_data(addr, 0x00, newmode)
        bus.write_byte_data(addr, 0xFE, prescale)
        bus.write_byte_data(addr, 0x00, oldmode)

        time.sleep(0.005)
        bus.write_byte_data(addr, 0x00, oldmode | 0x80)

    # 🔧 SET PWM
    def set_pwm(self, ch, us):
        val = int(us * 4096 / 20000)
        reg = 0x06 + 4 * ch

        self.bus.write_byte_data(self.addr, reg, 0)
        self.bus.write_byte_data(self.addr, reg+1, 0)
        self.bus.write_byte_data(self.addr, reg+2, val & 0xFF)
        self.bus.write_byte_data(self.addr, reg+3, val >> 8)

    # 🔥 ESC ARMING
    def arm_esc(self):
        self.get_logger().info("Arming ESC...")

        for _ in range(100):
            self.set_pwm(0, self.NEUTRAL)
            self.set_pwm(1, self.NEUTRAL)
            time.sleep(0.02)

        time.sleep(2)
        self.get_logger().info("ESC Armed ✅")

    # 🎮 CALLBACK (JOY → CMD_VEL)
    def callback(self, msg):
        self.last_time = time.time()

        lin = msg.linear.x
        ang = msg.angular.z

        # 🔥 DEADZONE
        if abs(lin) < self.deadzone:
            lin = 0.0
        if abs(ang) < self.deadzone:
            ang = 0.0

        # 🔥 DIFFERENTIAL MIX (SMOOTH)
        left = lin - (ang * self.turn_gain)
        right = lin + (ang * self.turn_gain)

        # Clamp
        left = max(-1.0, min(1.0, left))
        right = max(-1.0, min(1.0, right))

        # Scale to PWM
        self.left_pwm = int(self.NEUTRAL + left * self.scale)
        self.right_pwm = int(self.NEUTRAL + right * self.scale)

    # 🔁 LOOP
    def loop(self):

        # 🔥 WATCHDOG (AUTO STOP)
        if time.time() - self.last_time > self.timeout:
            self.left_pwm = self.NEUTRAL
            self.right_pwm = self.NEUTRAL

        self.set_pwm(0, self.left_pwm)
        self.set_pwm(1, self.right_pwm)

        self.get_logger().info(
            f"L:{self.left_pwm} R:{self.right_pwm}",
            throttle_duration_sec=1.0
        )

    # 🔥 SAFE SHUTDOWN
    def destroy_node(self):
        self.set_pwm(0, self.NEUTRAL)
        self.set_pwm(1, self.NEUTRAL)
        time.sleep(0.5)
        super().destroy_node()


def main():
    rclpy.init()
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
