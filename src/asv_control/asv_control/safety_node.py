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

        # 🔵 SAFE OUTPUT
        self.cmd_pub = self.create_publisher(
            Twist,
            '/asv/cmd_vel',
            10
        )

        # STATE
        self.inside = True
        self.last_cmd = Twist()

        # 🔥 TIME TRACKING
        self.last_cmd_time = self.get_clock().now()
        self.last_geo_time = self.get_clock().now()

        self.cmd_timeout = 1.0   # seconds
        self.geo_timeout = 1.0   # seconds

        # 🔥 TIMER LOOP
        self.timer = self.create_timer(0.1, self.safety_check)

        self.get_logger().info("🛡 Safety node (FAIL-SAFE) ACTIVE")

    # ---------------- GEOFENCE ----------------
    def geo_callback(self, msg):
        self.inside = msg.data
        self.last_geo_time = self.get_clock().now()

        if not self.inside:
            self.get_logger().warn(
                "🚨 GEOFENCE BREACH → STOP",
                throttle_duration_sec=2.0
            )

    # ---------------- CMD INPUT ----------------
    def cmd_callback(self, msg):
        self.last_cmd = msg
        self.last_cmd_time = self.get_clock().now()

    # ---------------- SAFETY CHECK LOOP ----------------
    def safety_check(self):
        now = self.get_clock().now()

        dt_cmd = (now - self.last_cmd_time).nanoseconds * 1e-9
        dt_geo = (now - self.last_geo_time).nanoseconds * 1e-9

        safe_cmd = Twist()

        # 🚨 CASE 1: NO COMMAND RECEIVED
        if dt_cmd > self.cmd_timeout:
            self.get_logger().warn(
                "⚠️ CMD TIMEOUT → STOP",
                throttle_duration_sec=2.0
            )
            self.cmd_pub.publish(safe_cmd)
            return

        # 🚨 CASE 2: GEOFENCE SIGNAL LOST
        if dt_geo > self.geo_timeout:
            self.get_logger().warn(
                "⚠️ GEOFENCE LOST → STOP",
                throttle_duration_sec=2.0
            )
            self.cmd_pub.publish(safe_cmd)
            return

        # 🚨 CASE 3: OUTSIDE GEOFENCE
        if not self.inside:
            self.cmd_pub.publish(safe_cmd)
            return

        # ✅ SAFE → PASS COMMAND
        self.cmd_pub.publish(self.last_cmd)


def main(args=None):
    rclpy.init(args=args)
    node = SafetyNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
