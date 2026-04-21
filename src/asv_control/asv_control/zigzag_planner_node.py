import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray, Pose, PoseStamped


class ZigZagPlanner(Node):
    def __init__(self):
        super().__init__('zigzag_planner')

        # 🔵 LOCAL POSE SUB
        self.sub = self.create_subscription(
            PoseStamped,
            '/asv/local_pose',
            self.pose_callback,
            10
        )

        # 🔵 WAYPOINT PUB
        self.pub = self.create_publisher(
            PoseArray,
            '/asv/waypoints',
            10
        )

        self.generated = False
        self.waypoints = []

        self.get_logger().info("🚀 ZigZag Planner (LOCAL FRAME) READY")

    def pose_callback(self, msg):
        if self.generated:
            return

        start_x = msg.pose.position.x
        start_y = msg.pose.position.y

        self.get_logger().info(f"📍 Start LOCAL: {start_x:.2f}, {start_y:.2f}")

        # 🔥 DEFINE AREA (meters)
        width = 15.0
        height = 15.0

        x_min = start_x - width / 2
        x_max = start_x + width / 2
        y_min = start_y - height / 2
        y_max = start_y + height / 2

        self.generate_zigzag(x_min, x_max, y_min, y_max)
        self.publish_waypoints()

        self.generated = True

    # ---------------- ZIGZAG ----------------
    def generate_zigzag(self, x_min, x_max, y_min, y_max):
        step = 2.0  # spacing in meters
        y = y_min
        toggle = True

        while y <= y_max:
            if toggle:
                x_range = [x_min, x_max]
            else:
                x_range = [x_max, x_min]

            for x in x_range:
                self.waypoints.append((x, y))

            toggle = not toggle
            y += step

        self.get_logger().info(f"✅ Generated {len(self.waypoints)} WPs")

    # ---------------- PUBLISH ----------------
    def publish_waypoints(self):
        msg = PoseArray()
        msg.header.frame_id = "map"
        msg.header.stamp = self.get_clock().now().to_msg()

        for x, y in self.waypoints:
            p = Pose()
            p.position.x = x
            p.position.y = y
            p.position.z = 0.0
            msg.poses.append(p)

        self.pub.publish(msg)

        self.get_logger().info("📡 Waypoints published!")


def main(args=None):
    rclpy.init(args=args)
    node = ZigZagPlanner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
