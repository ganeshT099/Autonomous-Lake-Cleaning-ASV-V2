import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Pose2D
from sensor_msgs.msg import Imu
from visualization_msgs.msg import Marker

import math


class BoatViz(Node):
    def __init__(self):
        super().__init__('boat_viz_node')

        # SUBSCRIBE LOCAL POSE
        self.pose_sub = self.create_subscription(
            Pose2D,
            '/asv/local_pose',
            self.pose_callback,
            10
        )

        # SUBSCRIBE IMU
        self.imu_sub = self.create_subscription(
            Imu,
            '/asv/imu/data',
            self.imu_callback,
            10
        )

        # PUBLISH MARKER
        self.marker_pub = self.create_publisher(
            Marker,
            '/asv/boat_viz',
            10
        )

        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

        self.get_logger().info("🚤 Boat Viz Started")

    def pose_callback(self, msg):
        self.x = msg.x
        self.y = msg.y

    def imu_callback(self, msg):
        q = msg.orientation

        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)

        self.yaw = math.atan2(siny, cosy)

        self.publish_marker()

    def publish_marker(self):
        marker = Marker()

        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()

        marker.ns = "boat"
        marker.id = 0
        marker.type = Marker.ARROW
        marker.action = Marker.ADD

        # POSITION
        marker.pose.position.x = self.x
        marker.pose.position.y = self.y
        marker.pose.position.z = 0.0

        # ORIENTATION
        marker.pose.orientation.z = math.sin(self.yaw / 2.0)
        marker.pose.orientation.w = math.cos(self.yaw / 2.0)

        # SIZE
        marker.scale.x = 1.5
        marker.scale.y = 0.4
        marker.scale.z = 0.4

        # COLOR
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 1.0

        self.marker_pub.publish(marker)


def main(args=None):
    rclpy.init(args=args)
    node = BoatViz()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
