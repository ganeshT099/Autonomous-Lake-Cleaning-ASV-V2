import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
import math

class ZigZagPlanner(Node):
    def __init__(self):
        super().__init__('zigzag_planner')

        from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

        qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE
        )

        # Subscribe GPS
        self.gps_sub = self.create_subscription(
            NavSatFix,
            '/asv/gps/fix',
            self.gps_callback,
            qos
        )

        self.generated = False
        self.waypoints = []

        self.get_logger().info("🚀 Dynamic ZigZag Planner Started")

    def gps_callback(self, msg):
        if self.generated:
            return

        lat = msg.latitude
        lon = msg.longitude

        self.get_logger().info(f"📍 Start Position: {lat:.6f}, {lon:.6f}")

        # 🔥 Create dynamic polygon (20m square)
        offset = 0.00015   # ~15–20 meters

        self.polygon = [
            (lat - offset, lon - offset),
            (lat - offset, lon + offset),
            (lat + offset, lon + offset),
            (lat + offset, lon - offset)
        ]

        self.generate_zigzag()
        self.generated = True

    # ---------------- POINT IN POLYGON ----------------
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

    # ---------------- ZIGZAG ----------------
    def generate_zigzag(self):
        lat_min = min(p[0] for p in self.polygon)
        lat_max = max(p[0] for p in self.polygon)
        lon_min = min(p[1] for p in self.polygon)
        lon_max = max(p[1] for p in self.polygon)

        step = 0.00003  # ~3m spacing
        lat = lat_min
        toggle = True

        while lat < lat_max:
            if toggle:
                lon_range = [lon_min, lon_max]
            else:
                lon_range = [lon_max, lon_min]

            for lon in lon_range:
                if self.point_in_polygon(lat, lon):
                    self.waypoints.append((lat, lon))

            toggle = not toggle
            lat += step

        self.get_logger().info(f"✅ Generated {len(self.waypoints)} waypoints")

        for wp in self.waypoints:
            self.get_logger().info(f"{wp[0]:.6f}, {wp[1]:.6f}")


def main(args=None):
    rclpy.init(args=args)
    node = ZigZagPlanner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
