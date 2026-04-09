import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

import time
import board
import busio
from adafruit_pca9685 import PCA9685


class ASVThruster(Node):

    def __init__(self):
        super().__init__('asv_thruster_node')

        # ===== CONFIG =====
        self.MIN_PWM = 1000
        self.MAX_PWM = 2000

        # 🔥 YOUR REAL NEUTRAL RANGE
        self.NEUTRAL_MIN = 1780
        self.NEUTRAL_MAX = 1800
        self.NEUTRAL_PWM = 1790   # center

        self.DEADBAND = 0.15
        self.TIMEOUT = 0.5

        self.last_cmd_time = time.time()

        # ===== I2C =====
        i2c = busio.I2C(board.SCL, board.SDA)
        self.pca = PCA9685(i2c)
        self.pca.frequency = 50

        self.left = self.pca.channels[0]
        self.right = self.pca.channels[1]

        # ===== ROS =====
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_callback,
            10
        )

        # failsafe timer
        self.timer = self.create_timer(0.1, self.safety_check)

        # ===== ARM ESC =====
        self.get_logger().info("🚀 Arming ESC (neutral hold)...")

        for _ in range(150):
            self.set_pwm(self.left, self.NEUTRAL_PWM)
            self.set_pwm(self.right, self.NEUTRAL_PWM)
            time.sleep(0.02)

        self.get_logger().info("✅ ESC armed. Holding neutral.")

    # ===== PWM =====
    def set_pwm(self, channel, us):
        pulse_length = 20000 / 4096
        pulse = int(us / pulse_length)
        channel.duty_cycle = pulse << 4

    # ===== MAPPING =====
    def map_to_pwm(self, val):

        # 🔥 HARD NEUTRAL ZONE
        if abs(val) < self.DEADBAND:
            return self.NEUTRAL_PWM

        # FORWARD
        if val > 0:
            return int(self.NEUTRAL_MAX + (self.MAX_PWM - self.NEUTRAL_MAX) * val)

        # REVERSE
        else:
            return int(self.NEUTRAL_MIN + (self.NEUTRAL_MIN - self.MIN_PWM) * val)

    # ===== CALLBACK =====
    def cmd_callback(self, msg):
        self.last_cmd_time = time.time()

        linear = msg.linear.x
        angular = msg.angular.z

        left_val = linear - angular
        right_val = linear + angular

        left_val = max(-1.0, min(1.0, left_val))
        right_val = max(-1.0, min(1.0, right_val))

        left_pwm = self.map_to_pwm(left_val)
        right_pwm = self.map_to_pwm(right_val)

        # 🔥 SNAP TO NEUTRAL RANGE
        if self.NEUTRAL_MIN <= left_pwm <= self.NEUTRAL_MAX:
            left_pwm = self.NEUTRAL_PWM

        if self.NEUTRAL_MIN <= right_pwm <= self.NEUTRAL_MAX:
            right_pwm = self.NEUTRAL_PWM

        self.set_pwm(self.left, left_pwm)
        self.set_pwm(self.right, right_pwm)

        self.get_logger().info(f"[THR] L={left_pwm} R={right_pwm}")

    # ===== FAILSAFE =====
    def safety_check(self):
        if time.time() - self.last_cmd_time > self.TIMEOUT:
            self.set_pwm(self.left, self.NEUTRAL_PWM)
            self.set_pwm(self.right, self.NEUTRAL_PWM)
            self.get_logger().info("⚠️ No input → Neutral")

    # ===== MAIN =====
def main(args=None):
    rclpy.init(args=args)
    node = ASVThruster()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
