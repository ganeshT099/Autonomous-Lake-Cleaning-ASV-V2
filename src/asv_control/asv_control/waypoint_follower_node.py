import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, Imu
from geometry_msgs.msg import Twist, PoseArray
import math

from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy


class WaypointFollower(Node):
    def __init__(self):
        super().__init__('waypoint_follower')

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

        # 🔵 SUBSCRIBE IMU
        self.imu_sub = self.create_subscription(
            Imu,
            '/asv/imu/data',
            self.imu_callback,
            qos
        )

        # 🔵 WAYPOINTS
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

        # STATE
        self.waypoints = []
        self.current_index = 0
        self.ready = False

        self.current_yaw = 0.0

        self.get_logger().info("🚀 Waypoint follower with IMU started")

    # ---------------- IMU CALLBACK ----------------
    def imu_callback(self, msg):
        # Integrate yaw (simple)
        self.current_yaw += msg.angular_velocity.z * 0.1

        # normalize (-pi to pi)
        self.current_yaw = math.atan2(
            math.sin(self.current_yaw),
            math.cos(self.current_yaw)
        )

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

        lat = msg.latitude
        lon = msg.longitude

        # ❌ Ignore invalid GPS
        if lat == 0.0 and lon == 0.0:
            return

        target_lat, target_lon = self.waypoints[self.current_index]

        distance = self.get_distance(lat, lon, target_lat, target_lon)
        desired_heading = self.get_bearing(lat, lon, target_lat, target_lon)

        # 🔥 KEY FIX
        heading_error = desired_heading - self.current_yaw

        # normalize error
        heading_error = math.atan2(
            math.sin(heading_error),
            math.cos(heading_error)
        )

        cmd = Twist()

        # 🎯 REACHED
        if distance < 1.5:
            self.get_logger().info(f"✅ Reached WP {self.current_index}")
            self.current_index += 1

            if self.current_index >= len(self.waypoints):
                self.get_logger().info("🏁 Mission Complete")
                cmd.linear.x = 0.0
                self.cmd_pub.publish(cmd)
                return

            return

        # 🚤 FORWARD SPEED
        cmd.linear.x = 0.5

        # 🔥 TURN CONTROL (IMU BASED)
        cmd.angular.z = max(min(heading_error * 1.5, 1.0), -1.0)

        self.cmd_pub.publish(cmd)

        self.get_logger().info(
            f"Dist:{distance:.2f} | Err:{heading_error:.2f} | Yaw:{self.current_yaw:.2f}"
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

    # ---------------- BEARING ----------------
    def get_bearing(self, lat1, lon1, lat2, lon2):
        y = math.sin(math.radians(lon2 - lon1)) * math.cos(math.radians(lat2))
        x = math.cos(math.radians(lat1)) * math.sin(math.radians(lat2)) - \
            math.sin(math.radians(lat1)) * math.cos(math.radians(lat2)) * \
            math.cos(math.radians(lon2 - lon1))

        return math.atan2(y, x)


def main(args=None):
    rclpy.init(args=args)
    node = WaypointFollower()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
