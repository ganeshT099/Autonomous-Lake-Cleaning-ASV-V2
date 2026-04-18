#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import PoseStamped

import math

from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy


class GPSToLocal(Node):
    def __init__(self):
        super().__init__('gps_to_local_node')

        # QoS (match GPS)
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

        # 🔵 PUBLISH LOCAL POSE
        self.pose_pub = self.create_publisher(
            PoseStamped,
            '/asv/local_pose',
            10
        )

        # 🔥 ORIGIN (set once)
        self.lat0 = None
        self.lon0 = None

        # 🔥 FLAGS
        self.origin_set = False

        self.get_logger().info("🌍 GPS → Local node started")

    # ---------------- GPS CALLBACK ----------------
    def gps_callback(self, msg):
        lat = msg.latitude
        lon = msg.longitude

        # ❌ Reject invalid GPS
        if lat == 0.0 and lon == 0.0:
            return

        if not (-90 <= lat <= 90 and -180 <= lon <= 180):
            return

        # ❌ Reject no-fix GPS
        if msg.status.status < 0:
            return

        # 🔥 SET ORIGIN ON FIRST VALID FIX
        if not self.origin_set:
            self.lat0 = lat
            self.lon0 = lon
            self.origin_set = True

            self.get_logger().info(
                f"📍 Origin locked at: {lat:.6f}, {lon:.6f}"
            )
            return

        # ---------------- CONVERSION ----------------
        # Flat Earth approximation (good for <1km)
        dlon = lon - self.lon0
        dlat = lat - self.lat0

        x = dlon * 111320 * math.cos(math.radians(self.lat0))
        y = dlat * 110540

        # ---------------- CREATE POSE ----------------
        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = "map"

        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = 0.0

        # Orientation default (no heading yet)
        pose.pose.orientation.w = 1.0

        # ---------------- PUBLISH ----------------
        self.pose_pub.publish(pose)

        # 🔥 Clean log (no spam)
        self.get_logger().info(
            f"📍 Local → x:{x:.2f} m | y:{y:.2f} m",
            throttle_duration_sec=1.0
        )


def main(args=None):
    rclpy.init(args=args)
    node = GPSToLocal()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
