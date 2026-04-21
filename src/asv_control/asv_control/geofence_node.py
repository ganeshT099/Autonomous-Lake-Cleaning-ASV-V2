import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped, Point
from visualization_msgs.msg import Marker
from std_msgs.msg import Bool


class GeofenceNode(Node):
    def __init__(self):
        super().__init__('geofence_node')

        # 🔥 GEOFENCE (LOCAL FRAME - meters)
        self.polygon = [
            (0.0, 0.0),
            (15.0, 0.0),
            (15.0, 10.0),
            (0.0, 10.0)
        ]

        # 🔥 SAFETY MARGIN (meters)
        self.margin = 1.0   # reduces false triggers

        # 🔵 SUBSCRIBE LOCAL POSE
        self.sub = self.create_subscription(
            PoseStamped,
            '/asv/local_pose',
            self.pose_callback,
            10
        )

        # 🔵 STATUS PUB
        self.pub = self.create_publisher(
            Bool,
            '/asv/geofence_status',
            10
        )

        # 🔵 VISUALIZATION
        self.marker_pub = self.create_publisher(
            Marker,
            '/asv/geofence_viz',
            10
        )

        self.get_logger().info("🚧 Geofence TEST node started")

    # ---------------- POINT IN POLYGON ----------------
    def point_in_polygon(self, x, y):
        inside = False
        j = len(self.polygon) - 1

        for i in range(len(self.polygon)):
            xi, yi = self.polygon[i]
            xj, yj = self.polygon[j]

            if ((yi > y) != (yj > y)) and \
               (x < (xj - xi) * (y - yi) / (yj - yi + 1e-9) + xi):
                inside = not inside

            j = i

        return inside

    # ---------------- DISTANCE TO EDGE (ADVANCED SAFETY) ----------------
    def near_boundary(self, x, y):
        for i in range(len(self.polygon)):
            x1, y1 = self.polygon[i]
            x2, y2 = self.polygon[(i+1) % len(self.polygon)]

            # line distance
            dx = x2 - x1
            dy = y2 - y1

            if dx == 0 and dy == 0:
                continue

            t = ((x - x1) * dx + (y - y1) * dy) / (dx*dx + dy*dy)
            t = max(0, min(1, t))

            proj_x = x1 + t * dx
            proj_y = y1 + t * dy

            dist = ((x - proj_x)**2 + (y - proj_y)**2) ** 0.5

            if dist < self.margin:
                return True

        return False

    # ---------------- RVIZ VISUAL ----------------
    def publish_geofence(self, inside):
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()

        marker.ns = "geofence"
        marker.id = 0
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD

        marker.scale.x = 0.15

        # 🔥 Color based on status
        if inside:
            marker.color.r = 0.0
            marker.color.g = 1.0
        else:
            marker.color.r = 1.0
            marker.color.g = 0.0

        marker.color.b = 0.0
        marker.color.a = 1.0

        marker.points = []

        for x, y in self.polygon:
            p = Point()
            p.x = x
            p.y = y
            p.z = 0.0
            marker.points.append(p)

        # close loop
        p = Point()
        p.x = self.polygon[0][0]
        p.y = self.polygon[0][1]
        p.z = 0.0
        marker.points.append(p)

        self.marker_pub.publish(marker)

    # ---------------- MAIN CALLBACK ----------------
    def pose_callback(self, msg):
        x = msg.pose.position.x
        y = msg.pose.position.y

        inside = self.point_in_polygon(x, y)
        near_edge = self.near_boundary(x, y)

        status = Bool()
        status.data = inside
        self.pub.publish(status)

        self.publish_geofence(inside)

        # 🔥 DEBUG LOGS
        if not inside:
            self.get_logger().warn(
                f"🚨 OUTSIDE | X:{x:.2f} Y:{y:.2f}",
                throttle_duration_sec=1.0
            )
        elif near_edge:
            self.get_logger().warn(
                f"⚠️ NEAR BOUNDARY | X:{x:.2f} Y:{y:.2f}",
                throttle_duration_sec=1.0
            )
        else:
            self.get_logger().info(
                f"✅ SAFE | X:{x:.2f} Y:{y:.2f}",
                throttle_duration_sec=2.0
            )


def main(args=None):
    rclpy.init(args=args)
    node = GeofenceNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
