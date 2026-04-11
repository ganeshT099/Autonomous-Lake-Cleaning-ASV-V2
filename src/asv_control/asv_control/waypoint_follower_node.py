import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import Twist, PoseArray
import math

from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy


class WaypointFollower(Node):
    def __init__(self):
        super().__init__('waypoint_follower')

        # ✅ QoS (MATCH GPS)
        qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE
        )

        # 🔵 SUBSCRIBE GPS
        self.gps_sub = self.create_subscription(
            NavSatFix,
            '/asv/gps/fix',
            self.gps_callback,
            qos
        )

        # 🔵 SUBSCRIBE WAYPOINTS
        self.wp_sub = self.create_subscription(
            PoseArray,
            '/asv/waypoints',
            self.wp_callback,
            10
        )

        # 🔵 CMD VEL PUB
        self.cmd_pub = self.create_publisher(
            Twist,
            '/asv/cmd_vel',
            10
        )

        # 🔥 INTERNAL STATE
        self.waypoints = []
        self.current_index = 0
        self.ready = False

        self.get_logger().info("🚀 Waypoint follower started")

    # ---------------- WAYPOINT CALLBACK ----------------
    def wp_callback(self, msg):
        self.waypoints = []

        for pose in msg.poses:
            lat = pose.position.x
            lon = pose.position.y
            self.waypoints.append((lat, lon))

        self.current_index = 0
        self.ready = True

        self.get_logger().info(f"📍 Received {len(self.waypoints)} waypoints")

    # ---------------- GPS CALLBACK ----------------
    def gps_callback(self, msg):
        if not self.ready or len(self.waypoints) == 0:
            return

        current_lat = msg.latitude
        current_lon = msg.longitude

        target_lat, target_lon = self.waypoints[self.current_index]

        distance = self.get_distance(current_lat, current_lon, target_lat, target_lon)
        heading_error = self.get_heading_error(current_lat, current_lon, target_lat, target_lon)

        cmd = Twist()

        # 🎯 Reached waypoint
        if distance < 1.5:
            self.get_logger().info(f"✅ Reached WP {self.current_index}")

            self.current_index += 1

            if self.current_index >= len(self.waypoints):
                self.get_logger().info("🏁 Mission Complete")
                cmd.linear.x = 0.0
                cmd.angular.z = 0.0
                self.cmd_pub.publish(cmd)
                return

            return

        # 🚤 Movement
        cmd.linear.x = 0.4

        # Smooth turning
        cmd.angular.z = max(min(heading_error * 0.8, 1.0), -1.0)

        self.cmd_pub.publish(cmd)

        self.get_logger().info(
            f"📍 Dist:{distance:.2f}m | HeadingErr:{heading_error:.2f} | WP:{self.current_index}"
        )

    # ---------------- DISTANCE ----------------
    def get_distance(self, lat1, lon1, lat2, lon2):
        R = 6371000
        phi1 = math.radians(lat1)
        phi2 = math.radians(lat2)

        dphi = math.radians(lat2 - lat1)
        dlambda = math.radians(lon2 - lon1)

        a = math.sin(dphi/2)**2 + \
            math.cos(phi1)*math.cos(phi2)*math.sin(dlambda/2)**2

        return R * (2 * math.atan2(math.sqrt(a), math.sqrt(1-a)))

    # ---------------- HEADING ----------------
    def get_heading_error(self, lat1, lon1, lat2, lon2):
        desired = math.atan2(lon2 - lon1, lat2 - lat1)
        return desired


def main(args=None):
    rclpy.init(args=args)
    node = WaypointFollower()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
