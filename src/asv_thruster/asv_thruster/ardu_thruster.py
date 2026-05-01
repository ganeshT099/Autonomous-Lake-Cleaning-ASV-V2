#!/usr/bin/env python3

import json, os, time, serial
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32MultiArray

NEUTRAL_FILE = os.path.expanduser("~/.asv_neutral.json")


class ASVThruster(Node):

    def __init__(self):
        super().__init__("asv_thruster_node")

        self.MIN_PWM  = 1000
        self.MAX_PWM  = 2000
        self.DEADBAND = 0.15
        self.TIMEOUT  = 0.5
        self.last_cmd_time = time.time()

        # Load saved neutrals (or fall back to 1500)
        saved = self._load_neutral()
        self.left_neutral  = saved.get("left",  1500)
        self.right_neutral = saved.get("right", 1500)
        self.get_logger().info(
            f"Neutral loaded: L={self.left_neutral}  R={self.right_neutral}")

        # Serial to Arduino
        try:
            self.ser = serial.Serial("/dev/ttyACM0", 115200, timeout=1)
            time.sleep(2)
            self.get_logger().info("✅ Arduino connected on /dev/ttyACM0")
        except Exception as e:
            self.get_logger().error(f"❌ Arduino not found: {e}")
            raise

        # ROS subscriptions
        self.create_subscription(Twist,           "/cmd_vel",                   self._cmd_cb,        10)
        self.create_subscription(Int32MultiArray, "/asv/thruster/direct_pwm",   self._direct_pwm_cb, 10)
        self.create_subscription(Int32MultiArray, "/asv/thruster/set_neutral",  self._set_neutral_cb,10)

        self.create_timer(0.1, self._failsafe)
        self.get_logger().info("🚀 Thruster node (Arduino) started")

    # ── Neutral persistence ───────────────────────────────────────────────────
    def _load_neutral(self):
        try:
            with open(NEUTRAL_FILE) as f:
                return json.load(f)
        except Exception:
            return {}

    def _save_neutral(self):
        try:
            with open(NEUTRAL_FILE, "w") as f:
                json.dump({"left": self.left_neutral, "right": self.right_neutral}, f)
        except Exception as e:
            self.get_logger().error(f"Save neutral failed: {e}")

    # ── Serial ────────────────────────────────────────────────────────────────
    def send_pwm(self, left, right):
        self.ser.write(f"{left},{right}\n".encode())

    # ── PWM mapping (per-thruster neutral) ────────────────────────────────────
    def _map(self, val, neutral):
        if abs(val) < self.DEADBAND:
            return neutral
        if val > 0:
            return int(neutral + (self.MAX_PWM - neutral) * val)
        return int(neutral + (neutral - self.MIN_PWM) * val)

    # ── cmd_vel callback ──────────────────────────────────────────────────────
    def _cmd_cb(self, msg):
        self.last_cmd_time = time.time()
        lv = max(-1.0, min(1.0, msg.linear.x - msg.angular.z))
        rv = max(-1.0, min(1.0, msg.linear.x + msg.angular.z))
        lp = self._map(lv, self.left_neutral)
        rp = self._map(rv, self.right_neutral)
        self.send_pwm(lp, rp)
        self.get_logger().info(f"[THR] L={lp} R={rp}")

    # ── Direct PWM (calibration test) ────────────────────────────────────────
    def _direct_pwm_cb(self, msg):
        if len(msg.data) >= 2:
            l = max(self.MIN_PWM, min(self.MAX_PWM, msg.data[0]))
            r = max(self.MIN_PWM, min(self.MAX_PWM, msg.data[1]))
            self.send_pwm(l, r)
            self.last_cmd_time = time.time()  # reset failsafe

    # ── Set & save neutral ────────────────────────────────────────────────────
    def _set_neutral_cb(self, msg):
        if len(msg.data) >= 2:
            self.left_neutral  = max(self.MIN_PWM, min(self.MAX_PWM, msg.data[0]))
            self.right_neutral = max(self.MIN_PWM, min(self.MAX_PWM, msg.data[1]))
            self._save_neutral()
            self.send_pwm(self.left_neutral, self.right_neutral)
            self.get_logger().info(
                f"✅ Neutral saved: L={self.left_neutral}  R={self.right_neutral}")

    # ── Failsafe ──────────────────────────────────────────────────────────────
    def _failsafe(self):
        if time.time() - self.last_cmd_time > self.TIMEOUT:
            self.send_pwm(self.left_neutral, self.right_neutral)

    def destroy_node(self):
        self.send_pwm(self.left_neutral, self.right_neutral)
        self.ser.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ASVThruster()
    try:
        rclpy.spin(node)
    except Exception:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
