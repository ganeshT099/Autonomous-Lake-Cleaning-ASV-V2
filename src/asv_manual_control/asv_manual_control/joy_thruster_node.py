#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
import smbus
import time


class JoyThrusterNode(Node):

    def __init__(self):
        super().__init__('joy_thruster_node')

        # 🔧 I2C (Jetson)
        self.bus = smbus.SMBus(7)
        self.addr = 0x40

        self._init_pwm()

        # 🔥 PWM SETTINGS
        self.NEUTRAL = 1500
        self.SCALE = 400
        self.BOOST = 1.5
        self.deadzone = 0.05

        # 🎮 CONTROLLER MAPPING (CONFIRMED)
        self.axis_left = 1   # Left stick vertical
        self.axis_right = 4  # Right stick vertical

        self.btn_deadman = 5  # RB
        self.btn_boost = 4    # LB

        # ROS
        self.sub = self.create_subscription(
            Joy, '/joy', self.joy_callback, 10)

        self.get_logger().info("🚤 FINAL JOYSTICK CONTROL READY")

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

    # 🔥 ARM ESC
    def arm_esc(self):
        self.get_logger().info("Arming ESC...")
        for _ in range(100):
            self.set_pwm(0, self.NEUTRAL)
            self.set_pwm(1, self.NEUTRAL)
            time.sleep(0.02)
        time.sleep(2)
        self.get_logger().info("ESC Armed ✅")

    # 🎮 MAIN CONTROL
    def joy_callback(self, msg):

        axes = msg.axes
        buttons = msg.buttons

        # 🔴 DEADMAN SWITCH (RB)
        if not buttons[self.btn_deadman]:
            self.set_pwm(0, self.NEUTRAL)
            self.set_pwm(1, self.NEUTRAL)
            return

        left = axes[self.axis_left]
        right = axes[self.axis_right]

        # 🔧 DEADZONE
        if abs(left) < self.deadzone:
            left = 0.0
        if abs(right) < self.deadzone:
            right = 0.0

        # 🔄 INVERT (so UP = forward)
        left = -left
        right = -right

        # ⚡ BOOST MODE (LB)
        if buttons[self.btn_boost]:
            left *= self.BOOST
            right *= self.BOOST

        # 🔒 CLAMP
        left = max(-1.0, min(1.0, left))
        right = max(-1.0, min(1.0, right))

        # 🎯 PWM OUTPUT
        left_pwm = int(self.NEUTRAL + left * self.SCALE)
        right_pwm = int(self.NEUTRAL + right * self.SCALE)

        self.set_pwm(0, left_pwm)
        self.set_pwm(1, right_pwm)

        self.get_logger().info(
            f"L:{left_pwm} R:{right_pwm}",
            throttle_duration_sec=1.0
        )

    # 🔥 SAFE STOP ON EXIT
    def destroy_node(self):
        self.set_pwm(0, self.NEUTRAL)
        self.set_pwm(1, self.NEUTRAL)
        time.sleep(0.5)
        super().destroy_node()


def main():
    rclpy.init()
    node = JoyThrusterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
