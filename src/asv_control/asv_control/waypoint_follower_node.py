import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped, Twist, PoseArray
from std_msgs.msg import Float64

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

        # 🔥 NEW → SUBSCRIBE FUSED HEADING
        self.heading_sub = self.create_subscription(
            Float64,
            '/asv/heading',
            self.heading_callback,
            10
        )

        # 🔵 WAYPOINTS
        self.wp_sub = self.create_subscription(
            PoseArray,
            '/asv/waypoints',
            self.wp_callback,
            10
        )

        # 🔵 CMD OUTPUT
        self.cmd_pub = self.create_publisher(
            Twist,
            '/asv/cmd_vel_raw',
            10
        )

        # STATE
        self.waypoints = []
        self.current_index = 0
        self.ready = False

        self.current_heading = 0.0  # 🔥 fused heading

        self.get_logger().info("🚀 Waypoint follower (FUSION MODE) started")

    # ---------------- HEADING CALLBACK ----------------
    def heading_callback(self, msg):
        self.current_heading = msg.data

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

        # 🔥 USE FUSED HEADING
        heading_error = desired_heading - self.current_heading
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

        # 🔥 TURN-IN-PLACE
        if abs(heading_error) > 0.6:
            cmd.linear.x = 0.0
        else:
            cmd.linear.x = 0.5 * (1 - abs(heading_error) / math.pi)

        # 🔥 ANGULAR CONTROL
        cmd.angular.z = max(min(heading_error * 1.5, 1.0), -1.0)

        self.cmd_pub.publish(cmd)

        # 🔍 DEBUG
        self.get_logger().info(
            f"Dist:{distance:.2f} | Err:{heading_error:.2f} | Heading:{self.current_heading:.2f}",
            throttle_duration_sec=1.0
        )


def main(args=None):
    rclpy.init(args=args)
    node = WaypointFollower()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
