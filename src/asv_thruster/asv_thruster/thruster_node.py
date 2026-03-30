#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import smbus2
import time
import threading
import signal
import sys

# ── Hardware ─────────────────────────────
I2C_BUS  = 7
PCA_ADDR = 0x40
LEFT_CH  = 0
RIGHT_CH = 1

# ── PWM (your ESC calibrated) ───────────
PWM_NEUTRAL  = 1500
PWM_MAX      = 1800
PWM_MIN      = 1250
PWM_MIN_SPIN = 1750
PWM_RANGE    = 300

CMD_TIMEOUT = 0.5
TURN_GAIN   = 0.7   # ⭐ smoother turning


class PCA9685:
    def __init__(self):
        self.bus = smbus2.SMBus(I2C_BUS)
        self.addr = PCA_ADDR
        self._init()

    def _init(self):
        self.bus.write_byte_data(self.addr, 0x00, 0x10)
        time.sleep(0.1)
        self.bus.write_byte_data(self.addr, 0xFE, 0x7A)
        time.sleep(0.1)
        self.bus.write_byte_data(self.addr, 0x00, 0x00)
        time.sleep(0.1)

    def set_pwm(self, ch, us):
        duty = int((us / 20000.0) * 4096)
        reg = 0x06 + ch * 4
        self.bus.write_byte_data(self.addr, reg, 0)
        self.bus.write_byte_data(self.addr, reg + 1, 0)
        self.bus.write_byte_data(self.addr, reg + 2, duty & 0xFF)
        self.bus.write_byte_data(self.addr, reg + 3, duty >> 8)

    def neutral(self):
        self.set_pwm(LEFT_CH, PWM_NEUTRAL)
        self.set_pwm(RIGHT_CH, PWM_NEUTRAL)


class ThrusterNode(Node):
    def __init__(self):
        super().__init__('asv_thruster_node')

        self.pca = PCA9685()
        self.left_pwm = PWM_NEUTRAL
        self.right_pwm = PWM_NEUTRAL
        self.target_left = PWM_NEUTRAL
        self.target_right = PWM_NEUTRAL
        self.last_cmd = time.time()
        self.armed = False
        self.lock = threading.Lock()

        # Arm ESC
        threading.Thread(target=self._arm, daemon=True).start()

        self.create_subscription(Twist, '/cmd_vel', self.cb, 10)
        self.status_pub = self.create_publisher(String, '/asv/thruster/status', 10)

        self.create_timer(0.05, self.loop)     # 20Hz
        self.create_timer(0.1, self.watchdog)

    def _arm(self):
        self.get_logger().info("Arming ESC...")
        for _ in range(60):
            self.pca.neutral()
            time.sleep(0.05)
        self.armed = True
        self.get_logger().info("ESC armed ✅")

    def to_pwm(self, val):
        if val > 0:
            pwm = PWM_NEUTRAL + int(val * PWM_RANGE)
            if val > 0.3:
                pwm = max(pwm, PWM_MIN_SPIN)
        elif val < 0:
            pwm = PWM_NEUTRAL + int(val * PWM_RANGE)
        else:
            pwm = PWM_NEUTRAL

        return max(PWM_MIN, min(PWM_MAX, pwm))

    def cb(self, msg):
        if not self.armed:
            return

        self.last_cmd = time.time()

        lin = msg.linear.x
        ang = msg.angular.z

        left  = lin - ang * TURN_GAIN
        right = lin + ang * TURN_GAIN

        left  = max(-1.0, min(1.0, left))
        right = max(-1.0, min(1.0, right))

        with self.lock:
            self.target_left  = self.to_pwm(left)
            self.target_right = self.to_pwm(right)

    def watchdog(self):
        if time.time() - self.last_cmd > CMD_TIMEOUT:
            with self.lock:
                self.target_left = PWM_NEUTRAL
                self.target_right = PWM_NEUTRAL

    def loop(self):
        if not self.armed:
            return

        step = 25  # faster response

        with self.lock:
            tl = self.target_left
            tr = self.target_right

        # ramp
        self.left_pwm += max(-step, min(step, tl - self.left_pwm))
        self.right_pwm += max(-step, min(step, tr - self.right_pwm))

        self.pca.set_pwm(LEFT_CH, self.left_pwm)
        self.pca.set_pwm(RIGHT_CH, self.right_pwm)

        s = String()
        s.data = f"L={self.left_pwm} R={self.right_pwm}"
        self.status_pub.publish(s)


def main():
    rclpy.init()
    node = ThrusterNode()
    rclpy.spin(node)


if __name__ == '__main__':
    main()
