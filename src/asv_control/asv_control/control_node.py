import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, NavSatFix
from geometry_msgs.msg import Twist
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy


class ASVControl(Node):
    def __init__(self):
        super().__init__('asv_control')

        # ✅ CORRECT QoS (MATCH IMU + GPS)
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=10
        )

        # SUBSCRIPTIONS
        self.imu_sub = self.create_subscription(
            Imu,
            '/asv/imu/data',
            self.imu_callback,
            qos
        )

        self.gps_sub = self.create_subscription(
            NavSatFix,
            '/asv/gps/fix',
            self.gps_callback,
            qos
        )

        # PUBLISHER
        self.cmd_pub = self.create_publisher(
            Twist,
            '/asv/cmd_vel',
            qos   # ✅ VERY IMPORTANT
        )

        # VARIABLES
        self.ax = 0.0
        self.lat = 0.0
        self.lon = 0.0

    def imu_callback(self, msg):
        self.ax = msg.linear_acceleration.x

    def gps_callback(self, msg):
        self.lat = msg.latitude
        self.lon = msg.longitude
        self.process()

    def process(self):
        cmd = Twist()

        self.get_logger().info(
            f"Lat:{self.lat:.6f} Lon:{self.lon:.6f} | ax:{self.ax:.2f}"
        )

        # 🚤 Chennai Geofence
        if (12.7 < self.lat < 13.0) and (80.0 < self.lon < 80.3):
            cmd.linear.x = 0.5
            self.get_logger().info("🚤 Moving forward")
        else:
            cmd.linear.x = 0.0
            self.get_logger().warn("🚨 Outside boundary!")

        self.cmd_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = ASVControl()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
