import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Bool


class GeofenceNode(Node):
    def __init__(self):
        super().__init__('geofence_node')

        # ✅ MATCH GPS NODE QoS EXACTLY
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=10
        )

        # 🔵 Polygon (edit later)
        self.polygon = [
            (12.8165, 80.0220),
            (12.8165, 80.0230),
            (12.8175, 80.0230),
            (12.8175, 80.0220)
        ]

        # ✅ SUBSCRIBER (MATCHED QoS)
        self.sub = self.create_subscription(
            NavSatFix,
            '/asv/gps/fix',
            self.gps_callback,
            sensor_qos
        )

        # ✅ PUBLISHER (same QoS for safety)
        self.pub = self.create_publisher(
            Bool,
            '/asv/geofence_status',
            sensor_qos
        )

        self.get_logger().info("🚧 Geofence node started")

    # ---------------- POLYGON CHECK ----------------
    def point_in_polygon(self, lat, lon):
        inside = False
        j = len(self.polygon) - 1

        for i in range(len(self.polygon)):
            xi, yi = self.polygon[i]
            xj, yj = self.polygon[j]

            if ((yi > lon) != (yj > lon)) and \
               (lat < (xj - xi) * (lon - yi) / (yj - yi + 1e-9) + xi):
                inside = not inside

            j = i

        return inside

    # ---------------- CALLBACK ----------------
    def gps_callback(self, msg):
        lat = msg.latitude
        lon = msg.longitude

        inside = self.point_in_polygon(lat, lon)

        status = Bool()
        status.data = inside
        self.pub.publish(status)

        # 🔥 Throttled log (avoid spam)
        if inside:
            self.get_logger().info(
                f"✅ INSIDE {lat:.6f}, {lon:.6f}",
                throttle_duration_sec=2.0
            )
        else:
            self.get_logger().warn(
                f"🚨 OUTSIDE {lat:.6f}, {lon:.6f}",
                throttle_duration_sec=2.0
            )


def main(args=None):
    rclpy.init(args=args)
    node = GeofenceNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
