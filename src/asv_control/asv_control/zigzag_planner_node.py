import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import PoseArray, Pose
import math

from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy


class ZigZagPlanner(Node):
    def __init__(self):
        super().__init__('zigzag_planner')

        qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE
        )

        # GPS SUB
        self.gps_sub = self.create_subscription(
            NavSatFix,
            '/asv/gps/fix',
            self.gps_callback,
            qos
        )

        # WAYPOINT PUB
        self.wp_pub = self.create_publisher(
            PoseArray,
            '/asv/waypoints',
            10
        )

        self.generated = False
        self.waypoints = []

        self.get_logger().info("🚀 ZigZag Planner READY")

    def gps_callback(self, msg):
        if self.generated:
            return

        lat = msg.latitude
        lon = msg.longitude

        self.get_logger().info(f"📍 Start: {lat:.6f}, {lon:.6f}")

        # 🔥 15m boundary
        offset = 0.00015

        self.polygon = [
            (lat - offset, lon - offset),
            (lat - offset, lon + offset),
            (lat + offset, lon + offset),
            (lat + offset, lon - offset)
        ]

        self.generate_zigzag()
        self.publish_waypoints()

        self.generated = True

    # ---------------- ZIGZAG ----------------
    def generate_zigzag(self):
        lat_min = min(p[0] for p in self.polygon)
        lat_max = max(p[0] for p in self.polygon)
        lon_min = min(p[1] for p in self.polygon)
        lon_max = max(p[1] for p in self.polygon)

        step = 0.00003
        lat = lat_min
        toggle = True

        while lat < lat_max:
            if toggle:
                lon_range = [lon_min, lon_max]
            else:
                lon_range = [lon_max, lon_min]

            for lon in lon_range:
                self.waypoints.append((lat, lon))

            toggle = not toggle
            lat += step

        self.get_logger().info(f"✅ Generated {len(self.waypoints)} WPs")

    # ---------------- PUBLISH ----------------
    def publish_waypoints(self):
        msg = PoseArray()
        msg.header.frame_id = "map"
        msg.header.stamp = self.get_clock().now().to_msg()

        for lat, lon in self.waypoints:
            p = Pose()
            p.position.x = lat
            p.position.y = lon
            p.position.z = 0.0
            msg.poses.append(p)

        self.wp_pub.publish(msg)

        self.get_logger().info("📡 Waypoints published!")


def main(args=None):
    rclpy.init(args=args)
    node = ZigZagPlanner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
