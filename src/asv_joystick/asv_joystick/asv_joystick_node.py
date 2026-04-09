import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist


class ASVJoystick(Node):

    def __init__(self):
        super().__init__('asv_joystick_node')

        self.create_subscription(Joy, '/joy', self.joy_callback, 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.get_logger().info("🚀 Joystick FINAL FIXED")

    def deadzone(self, val, th=0.1):
        return 0.0 if abs(val) < th else val

    def normalize_trigger(self, val):
        return (1 - val) / 2

    def expo(self, x):
        return x * x * (1 if x >= 0 else -1)

    def joy_callback(self, msg):
        cmd = Twist()

        rt_raw = msg.axes[5]
        lt_raw = msg.axes[2]
        steering_raw = msg.axes[3]
        rb = msg.buttons[5]

        # NORMALIZE
        forward = self.normalize_trigger(rt_raw)
        reverse = self.normalize_trigger(lt_raw)

        # REMOVE NOISE
        if forward < 0.1:
            forward = 0.0
        if reverse < 0.1:
            reverse = 0.0

        # COMBINE
        linear = forward - reverse

        # 🔥 BOOST (CRITICAL FIX)
        if linear > 0:
            linear = 0.6 + 0.4 * linear
        elif linear < 0:
            linear = -0.6 + 0.4 * linear

        # EXPO
        linear = self.expo(linear)

        # STEERING
        steering = self.deadzone(steering_raw, 0.1)

        # DEADMAN
        if rb == 0:
            linear = 0.0
            steering = 0.0
            self.get_logger().warn("🛑 STOP")

        cmd.linear.x = float(linear)
        cmd.angular.z = float(steering)

        self.cmd_pub.publish(cmd)

        self.get_logger().info(
            f"LIN:{linear:.2f} RT:{rt_raw:.2f} LT:{lt_raw:.2f}"
        )


def main():
    rclpy.init()
    node = ASVJoystick()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
