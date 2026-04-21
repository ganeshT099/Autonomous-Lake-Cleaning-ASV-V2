import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped, Twist, PoseArray
from sensor_msgs.msg import Imu

import math


class WaypointFollower(Node):
    def __init__(self):
        super().__init__('waypoint_follower')

        # 🔵 SUBSCRIBE LOCAL POSITION
        self.pose_sub = self.create_subscription(
            PoseStamped,
            '/asv/local_pose',
            self.pose_callback,
            10
        )

        # 🔵 SUBSCRIBE IMU
        self.imu_sub = self.create_subscription(
            Imu,
            '/asv/imu/data',
            self.imu_callback,
            10
        )

        # 🔵 WAYPOINTS (LOCAL FRAME)
        self.wp_sub = self.create_subscription(
            PoseArray,
            '/asv/waypoints',
            self.wp_callback,
            10
        )

        # 🔵 PUBLISH RAW CMD (goes to safety node)
        self.cmd_pub = self.create_publisher(
            Twist,
            '/asv/cmd_vel_raw',
            10
        )

        # STATE
        self.waypoints = []
        self.current_index = 0
        self.ready = False

        self.current_yaw = 0.0

        self.get_logger().info("🚀 Waypoint follower (LOCAL FRAME) started")

    # ---------------- IMU CALLBACK ----------------
    def imu_callback(self, msg):
        # ⚠️ Simple integration (works but drifts slowly)
        dt = 0.1
        self.current_yaw += msg.angular_velocity.z * dt

        # normalize [-pi, pi]
        self.current_yaw = math.atan2(
            math.sin(self.current_yaw),
            math.cos(self.current_yaw)
        )

    # ---------------- WAYPOINT CALLBACK ----------------
    def wp_callback(self, msg):
        self.waypoints = []

        for pose in msg.poses:
            x = pose.position.x
            y = pose.position.y
            self.waypoints.append((x, y))

        self.current_index = 0
        self.ready = True

        self.get_logger().info(f"📍 Received {len(self.waypoints)} waypoints")

    # ---------------- MAIN CONTROL ----------------
    def pose_callback(self, msg):
        if not self.ready or len(self.waypoints) == 0:
            return

        x = msg.pose.position.x
        y = msg.pose.position.y

        target_x, target_y = self.waypoints[self.current_index]

        dx = target_x - x
        dy = target_y - y

        distance = math.sqrt(dx * dx + dy * dy)
        desired_heading = math.atan2(dy, dx)

        heading_error = desired_heading - self.current_yaw
        heading_error = math.atan2(
            math.sin(heading_error),
            math.cos(heading_error)
        )

        cmd = Twist()

        # 🎯 WAYPOINT REACHED
        if distance < 1.5:
            self.get_logger().info(f"✅ Reached WP {self.current_index}")
            self.current_index += 1

            if self.current_index >= len(self.waypoints):
                self.get_logger().info("🏁 Mission Complete")
                cmd.linear.x = 0.0
                cmd.angular.z = 0.0
                self.cmd_pub.publish(cmd)
                return

            return

        # 🔥 TURN-IN-PLACE (important for sharp turns)
        if abs(heading_error) > 0.6:
            cmd.linear.x = 0.0
        else:
            # 🔥 SMOOTH SPEED CONTROL
            cmd.linear.x = 0.5 * (1 - abs(heading_error) / math.pi)

        # 🔥 ANGULAR CONTROL (P controller)
        cmd.angular.z = max(min(heading_error * 1.5, 1.0), -1.0)

        self.cmd_pub.publish(cmd)

        # 🔍 DEBUG
        self.get_logger().info(
            f"Dist:{distance:.2f} | Err:{heading_error:.2f} | Yaw:{self.current_yaw:.2f}",
            throttle_duration_sec=1.0
        )


def main(args=None):
    rclpy.init(args=args)
    node = WaypointFollower()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
