import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, NavSatFix
from std_msgs.msg import Float64
import math


class HeadingFusion(Node):
    def __init__(self):
        super().__init__('heading_fusion')

        # Subscribers
        self.imu_sub = self.create_subscription(
            Imu, '/asv/imu/data', self.imu_callback, 10)

        self.gps_sub = self.create_subscription(
            NavSatFix, '/asv/gps/fix', self.gps_callback, 10)

        # Publisher
        self.pub = self.create_publisher(
            Float64, '/asv/heading', 10)

        # Variables
        self.imu_yaw = 0.0

        self.prev_lat = None
        self.prev_lon = None
        self.gps_heading = 0.0
        self.speed = 0.0

        self.alpha = 0.7  # fusion weight

        self.get_logger().info("🚀 Heading Fusion Node Started")

    # ---------------- IMU ----------------
    def imu_callback(self, msg):
        q = msg.orientation

        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)

        self.imu_yaw = math.atan2(siny_cosp, cosy_cosp)

    # ---------------- GPS ----------------
    def gps_callback(self, msg):
        lat = msg.latitude
        lon = msg.longitude

        if self.prev_lat is None:
            self.prev_lat = lat
            self.prev_lon = lon
            return

        # distance approx (meters)
        dx = (lon - self.prev_lon) * 111320 * math.cos(math.radians(lat))
        dy = (lat - self.prev_lat) * 111320

        distance = math.sqrt(dx*dx + dy*dy)

        # speed estimation
        self.speed = distance / 0.1  # assuming ~10Hz

        if distance > 0.2:  # ignore noise
            self.gps_heading = math.atan2(dy, dx)

        self.prev_lat = lat
        self.prev_lon = lon

        self.fuse()

    # ---------------- FUSION ----------------
    def fuse(self):
        heading = Float64()

        # 🟢 Moving → trust GPS
        if self.speed > 0.3:
            fused = self.alpha * self.gps_heading + \
                    (1 - self.alpha) * self.imu_yaw
        else:
            # 🔵 Stationary → trust IMU
            fused = self.imu_yaw

        # normalize
        fused = math.atan2(math.sin(fused), math.cos(fused))

        heading.data = fused
        self.pub.publish(heading)

        self.get_logger().info(
            f"IMU:{self.imu_yaw:.2f} GPS:{self.gps_heading:.2f} → FUSED:{fused:.2f}",
            throttle_duration_sec=1.0
        )


def main(args=None):
    rclpy.init(args=args)
    node = HeadingFusion()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
