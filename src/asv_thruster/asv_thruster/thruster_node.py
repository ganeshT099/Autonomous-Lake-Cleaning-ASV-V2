#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time
from smbus2 import SMBus

# PCA9685 Registers
PCA9685_ADDR = 0x40
MODE1 = 0x00
PRESCALE = 0xFE
LED0_ON_L = 0x06


class PCA9685:
    def __init__(self, bus=7, address=0x40):
        self.bus = SMBus(bus)
        self.addr = address
        self.set_pwm_freq(50)

    def write(self, reg, value):
        self.bus.write_byte_data(self.addr, reg, value)

    def set_pwm_freq(self, freq_hz):
        prescale = int(25000000.0 / (4096 * freq_hz) - 1)
        oldmode = self.bus.read_byte_data(self.addr, MODE1)
        self.write(MODE1, (oldmode & 0x7F) | 0x10)
        self.write(PRESCALE, prescale)
        self.write(MODE1, oldmode)
        time.sleep(0.005)
        self.write(MODE1, oldmode | 0x80)

    def set_pwm(self, channel, on, off):
        reg = LED0_ON_L + 4 * channel
        self.bus.write_i2c_block_data(
            self.addr, reg,
            [on & 0xFF, on >> 8, off & 0xFF, off >> 8]
        )


class ThrusterNode(Node):

    # ✅ YOUR ESC CALIBRATION
    PWM_MIN = 1250
    PWM_NEUTRAL = 1500
    PWM_MAX = 1800
    PWM_RANGE = 300

    def __init__(self):
        super().__init__('asv_thruster_node')

        self.pca = PCA9685()

        self.LEFT = 0
        self.RIGHT = 1

        # 🔥 STRONG ARMING SIGNAL
        self.get_logger().info("Arming ESC...")

        for _ in range(100):
            self.set_pwm_us(self.LEFT, self.PWM_NEUTRAL)
            self.set_pwm_us(self.RIGHT, self.PWM_NEUTRAL)
            time.sleep(0.05)

        self.create_subscription(
            Twist,
            '/cmd_vel',
            self.callback,
            10
        )

        self.get_logger().info("🚤 Thruster READY (CALIBRATED)")

    # ---------------- PWM ----------------

    def us_to_counts(self, us):
        return int(us * 4096 / 20000)

    def set_pwm_us(self, channel, us):
        counts = self.us_to_counts(us)
        self.pca.set_pwm(channel, 0, counts)

    # ---------------- CONTROL ----------------

    def callback(self, msg):

        linear = msg.linear.x
        angular = msg.angular.z

        # Differential drive
        left = linear - angular
        right = linear + angular

        # Convert to PWM (based on your ESC)
        pwm_left = int(self.PWM_NEUTRAL + left * self.PWM_RANGE)
        pwm_right = int(self.PWM_NEUTRAL + right * self.PWM_RANGE)

        # Clamp to valid range
        pwm_left = max(self.PWM_MIN, min(self.PWM_MAX, pwm_left))
        pwm_right = max(self.PWM_MIN, min(self.PWM_MAX, pwm_right))

        # Send signal
        self.set_pwm_us(self.LEFT, pwm_left)
        self.set_pwm_us(self.RIGHT, pwm_right)

        self.get_logger().info(f"L:{pwm_left} R:{pwm_right}")


def main():
    rclpy.init()
    node = ThrusterNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
