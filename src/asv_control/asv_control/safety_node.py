import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool


class SafetyNode(Node):
    def __init__(self):
        super().__init__('safety_node')

        # 🔵 SUBSCRIBE RAW COMMAND
        self.cmd_sub = self.create_subscription(
            Twist,
            '/asv/cmd_vel_raw',
            self.cmd_callback,
            10
        )

        # 🔵 SUBSCRIBE GEOFENCE STATUS
        self.geo_sub = self.create_subscription(
            Bool,
            '/asv/geofence_status',
            self.geo_callback,
            10
        )

        # 🔵 SAFE OUTPUT TO THRUSTER
        self.cmd_pub = self.create_publisher(
            Twist,
            '/asv/cmd_vel',
            10
        )

        # STATE
        self.inside = True
        self.last_cmd = Twist()

        self.get_logger().info("🛡 Safety node ACTIVE")

    # ---------------- GEOFENCE ----------------
    def geo_callback(self, msg):
        self.inside = msg.data

        if not self.inside:
            self.get_logger().warn(
                "🚨 GEOFENCE BREACH → STOPPING",
                throttle_duration_sec=2.0
            )

    # ---------------- CMD ----------------
    def cmd_callback(self, msg):
        self.last_cmd = msg

        safe_cmd = Twist()

        if self.inside:
            # ✅ PASS THROUGH
            safe_cmd = msg
        else:
            # 🚨 FORCE STOP
            safe_cmd.linear.x = 0.0
            safe_cmd.angular.z = 0.0

        self.cmd_pub.publish(safe_cmd)


def main(args=None):
    rclpy.init(args=args)
    node = SafetyNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
