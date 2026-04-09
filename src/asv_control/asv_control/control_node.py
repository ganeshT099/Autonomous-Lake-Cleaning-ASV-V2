import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, NavSatFix
from geometry_msgs.msg import Twist

class ASVControl(Node):
    def __init__(self):
        super().__init__('asv_control')

        # 🔹 Subscribers
        self.imu_sub = self.create_subscription(
            Imu,
            '/asv/imu/data',
            self.imu_callback,
            10)

        self.gps_sub = self.create_subscription(
            NavSatFix,
            '/asv/fix',
            self.gps_callback,
            10)

        # 🔹 Publisher
        self.cmd_pub = self.create_publisher(
            Twist,
            '/asv/cmd_vel',
            10)

        # Data
        self.ax = 0.0
        self.ay = 0.0
        self.az = 0.0
        self.lat = 0.0
        self.lon = 0.0

    def imu_callback(self, msg):
        self.ax = msg.linear_acceleration.x
        self.ay = msg.linear_acceleration.y
        self.az = msg.linear_acceleration.z

    def gps_callback(self, msg):
        self.lat = msg.latitude
        self.lon = msg.longitude
        self.process()

    def process(self):
        cmd = Twist()

        # 🚤 DEFAULT FORWARD MOTION
        cmd.linear.x = 0.5
        cmd.angular.z = 0.0

        # ⚠️ Disturbance safety
        if abs(self.ax) > 5 or abs(self.ay) > 5:
            cmd.linear.x = 0.0
            self.get_logger().warn("⚠️ Disturbance! Stopping")

        # 🌍 GEOFENCE (UPDATED FOR YOUR CURRENT GPS REGION)
        if not (36.0 < self.lat < 39.0 and -123.5 < self.lon < -121.0):
            cmd.linear.x = 0.0
            self.get_logger().warn("🚨 Outside boundary!")

        # 🔥 CLEAN LOG (no spam)
        self.get_logger().info(
            f"Lat:{self.lat:.5f} Lon:{self.lon:.5f} | Move:{cmd.linear.x}",
            throttle_duration_sec=1.0
        )

        # Publish
        self.cmd_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = ASVControl()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
