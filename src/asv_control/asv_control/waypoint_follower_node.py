import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point, PoseStamped
from std_msgs.msg import Bool

import math


class GeofenceNode(Node):
    def __init__(self):
        super().__init__('geofence_node')

        # 🔥 PARAMETERS
        self.radius = 12.0   # meters (change here: 10–15)

        # 🔵 SUBSCRIBE LOCAL POSITION
        self.sub = self.create_subscription(
            PoseStamped,
            '/asv/local_pose',
            self.pose_callback,
            10
        )

        # 🔵 STATUS PUB
        self.status_pub = self.create_publisher(
            Bool,
            '/asv/geofence_status',
            10
        )

        # 🔵 RVIZ MARKER PUB
        self.marker_pub = self.create_publisher(
            Marker,
            '/asv/geofence_viz',
            10
        )

        self.get_logger().info(f"🚧 Circular Geofence ACTIVE (R={self.radius}m)")

    # ---------------- CHECK ----------------
    def is_inside(self, x, y):
        distance = math.sqrt(x**2 + y**2)
        return distance <= self.radius

    # ---------------- VISUAL ----------------
    def publish_circle(self):
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()

        marker.ns = "geofence"
        marker.id = 0
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD

        marker.scale.x = 0.1  # line thickness

        # 🔥 RED circle
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0

        marker.points = []

        num_points = 50

        for i in range(num_points + 1):
            angle = 2 * math.pi * i / num_points
            x = self.radius * math.cos(angle)
            y = self.radius * math.sin(angle)

            p = Point()
            p.x = x
            p.y = y
            p.z = 0.0

            marker.points.append(p)

        self.marker_pub.publish(marker)

    # ---------------- CALLBACK ----------------
    def pose_callback(self, msg):
        x = msg.pose.position.x
        y = msg.pose.position.y

        inside = self.is_inside(x, y)

        status = Bool()
        status.data = inside
        self.status_pub.publish(status)

        # 🔥 Always visualize
        self.publish_circle()

        if inside:
            self.get_logger().info(
                f"✅ INSIDE ({x:.2f}, {y:.2f})",
                throttle_duration_sec=2.0
            )
        else:
            self.get_logger().warn(
                f"🚨 OUTSIDE ({x:.2f}, {y:.2f})",
                throttle_duration_sec=2.0
            )


def main(args=None):
    rclpy.init(args=args)
    node = GeofenceNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
