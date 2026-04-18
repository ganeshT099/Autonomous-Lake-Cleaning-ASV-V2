import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Imu
from visualization_msgs.msg import Marker
import math


class BoatViz(Node):
    def __init__(self):
        super().__init__('boat_viz_node')

        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

        self.create_subscription(PoseStamped, '/asv/local_pose', self.pose_cb, 10)
        self.create_subscription(Imu, '/asv/imu/data', self.imu_cb, 10)

        self.pub = self.create_publisher(Marker, '/asv/boat_viz', 10)

        self.timer = self.create_timer(0.1, self.publish_marker)

        self.get_logger().info("🚤 Boat visualization node started")

    def pose_cb(self, msg):
        self.x = msg.pose.position.x
        self.y = msg.pose.position.y

    def imu_cb(self, msg):
        # extract yaw from angular velocity (approx integration skipped)
        self.yaw += msg.angular_velocity.z * 0.1

    def publish_marker(self):
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()

        marker.type = Marker.ARROW
        marker.action = Marker.ADD

        marker.scale.x = 1.0
        marker.scale.y = 0.3
        marker.scale.z = 0.3

        marker.color.r = 0.0
        marker.color.g = 0.5
        marker.color.b = 1.0
        marker.color.a = 1.0

        marker.pose.position.x = self.x
        marker.pose.position.y = self.y
        marker.pose.position.z = 0.0

        # yaw → quaternion
        qz = math.sin(self.yaw / 2)
        qw = math.cos(self.yaw / 2)

        marker.pose.orientation.z = qz
        marker.pose.orientation.w = qw

        self.pub.publish(marker)


def main(args=None):
    rclpy.init(args=args)
    node = BoatViz()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
