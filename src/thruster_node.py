#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time

from adafruit_pca9685 import PCA9685
import board
import busio


class ThrusterNode(Node):

    def __init__(self):
        super().__init__('thruster_node')

        # 🔌 I2C INIT
        i2c = busio.I2C(board.SCL, board.SDA)
        self.pca = PCA9685(i2c)
        self.pca.frequency = 50

        self.LEFT = 0
        self.RIGHT = 1

        # 🔥 ESC CALIBRATION
        self.NEUTRAL = 1500
        self.FWD_RANGE = 350
        self.REV_RANGE = 250

        # 🔧 CONTROL PARAMS
        self.max_step = 15     # ramping speed
        self.left_trim = 1.0
        self.right_trim = 0.97

        # 🔄 STATE
        self.left_pwm = 1500
        self.right_pwm = 1500
        self.prev_left_pwm = 1500
        self.prev_right_pwm = 1500

        self.lin_prev = 0.0
        self.ang_prev = 0.0

        self.last_time = time.time()
        self.timeout = 0.5

        # ROS
        self.sub = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_callback, 10)

        self.timer = self.create_timer(0.05, self.control_loop)

        self.get_logger().info("🚤 10/10 Thruster Node Started")

        # 🔥 ESC ARMING
        self.arm_esc()

    # 🔴 ESC ARMING
    def arm_esc(self):
        self.get_logger().info("Arming ESC...")
        start = time.time()
        while time.time() - start < 3:
            self.set_pwm(self.LEFT, self.NEUTRAL)
            self.set_pwm(self.RIGHT, self.NEUTRAL)
            time.sleep(0.1)
        self.get_logger().info("ESC Armed ✅")

    # 🔹 Deadzone
    def deadzone(self, val):
        return 0.0 if abs(val) < 0.05 else val

    # 🔹 Nonlinear response
    def shape(self, val):
        return val * abs(val)

    # 🔹 PWM mapping
    def map_pwm(self, val):
        if val > 0:
            return int(self.NEUTRAL + val * self.FWD_RANGE)
        else:
            return int(self.NEUTRAL + val * self.REV_RANGE)

    # 🔹 Ramp limiter
    def ramp(self, target, current):
        if target > current:
            return min(target, current + self.max_step)
        else:
            return max(target, current - self.max_step)

    # 🔹 PWM conversion
    def us_to_duty(self, us):
        pulse = 1000000 / 50 / 4096
        duty = int(us / pulse)
        return max(0, min(65535, duty))

    def set_pwm(self, ch, us):
        self.pca.channels[ch].duty_cycle = self.us_to_duty(us)

    # 🔹 Callback
    def cmd_callback(self, msg):
        self.last_time = time.time()

        lin = self.deadzone(msg.linear.x)
        ang = self.deadzone(msg.angular.z)

        # 🔥 FILTER
        lin = 0.7 * self.lin_prev + 0.3 * lin
        ang = 0.7 * self.ang_prev + 0.3 * ang

        self.lin_prev = lin
        self.ang_prev = ang

        # 🔥 NONLINEAR
        lin = self.shape(lin)
        ang = self.shape(ang)

        # 🔥 SMART TURN
        turn_scale = (1 - abs(lin))
        left = lin - ang * turn_scale
        right = lin + ang * turn_scale

        # 🔧 TRIM
        left *= self.left_trim
        right *= self.right_trim

        # CLAMP
        left = max(-1, min(1, left))
        right = max(-1, min(1, right))

        self.left_pwm = self.map_pwm(left)
        self.right_pwm = self.map_pwm(right)

    # 🔹 CONTROL LOOP
    def control_loop(self):

        # WATCHDOG
        if time.time() - self.last_time > self.timeout:
            self.left_pwm = self.NEUTRAL
            self.right_pwm = self.NEUTRAL

        # 🔥 RAMPING
        self.left_pwm = self.ramp(self.left_pwm, self.prev_left_pwm)
        self.right_pwm = self.ramp(self.right_pwm, self.prev_right_pwm)

        self.prev_left_pwm = self.left_pwm
        self.prev_right_pwm = self.right_pwm

        # SEND PWM
        self.set_pwm(self.LEFT, self.left_pwm)
        self.set_pwm(self.RIGHT, self.right_pwm)

        # DEBUG (1 sec)
        self.get_logger().info(
            f"L:{self.left_pwm} R:{self.right_pwm}",
            throttle_duration_sec=1.0
        )

    def destroy_node(self):
        self.set_pwm(self.LEFT, self.NEUTRAL)
        self.set_pwm(self.RIGHT, self.NEUTRAL)
        self.pca.deinit()
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
