#!/usr/bin/env python3
"""
ASV Localization Node — Full Multi-Sensor EKF Fusion
Sensors: GPS + IMU + mmWave Radar + 3x Ultrasonics + Camera
Publishes:
  /asv/localization/pose    (geometry_msgs/PoseStamped)
  /asv/localization/odom    (nav_msgs/Odometry)
  /asv/localization/status  (std_msgs/String)
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import NavSatFix, Imu, Range, Image
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, TransformStamped
from std_msgs.msg import String, Float32
import tf2_ros
import numpy as np
import math
from pyproj import Proj
import cv2
from cv_bridge import CvBridge


class MultiSensorEKF:
    """
    EKF State: [x, y, yaw, vx, vy]
    x, y   = local metres from origin (first GPS fix)
    yaw    = heading in radians
    vx, vy = velocity in m/s
    """
    def __init__(self):
        self.x = np.zeros(5)
        self.P = np.eye(5) * 1.0

        # Process noise — tuned for ASV on water
        self.Q = np.diag([0.05, 0.05, 0.005, 0.2, 0.2])

        # Measurement noise per sensor
        self.R_gps        = np.diag([2.0, 2.0])         # GPS x,y metres
        self.R_imu        = np.array([[0.01]])            # IMU yaw radians
        self.R_radar      = np.array([[0.05]])            # Radar distance metres
        self.R_ultrasonic = np.diag([0.02, 0.02, 0.02]) # F,R,L ultrasonics
        self.R_camera     = np.diag([0.3, 0.3])          # Visual odometry x,y

        self.initialized = False

    def predict(self, dt):
        if dt <= 0 or dt > 1.0:
            return
        x, y, yaw, vx, vy = self.x
        self.x[0] += vx * math.cos(yaw) * dt
        self.x[1] += vx * math.sin(yaw) * dt
        # Jacobian F
        F = np.eye(5)
        F[0][2] = -vx * math.sin(yaw) * dt
        F[0][3] =  math.cos(yaw) * dt
        F[1][2] =  vx * math.cos(yaw) * dt
        F[1][3] =  math.sin(yaw) * dt
        self.P = F @ self.P @ F.T + self.Q

    def _update(self, z, H, R):
        """Generic EKF update step"""
        y = z - H @ self.x
        # Wrap yaw residuals
        if H.shape[0] == 1 and H[0][2] == 1.0:
            y[0] = math.atan2(math.sin(y[0]), math.cos(y[0]))
        S = H @ self.P @ H.T + R
        K = self.P @ H.T @ np.linalg.inv(S)
        self.x = self.x + K @ y
        self.P = (np.eye(5) - K @ H) @ self.P

    def update_gps(self, gps_x, gps_y):
        H = np.zeros((2, 5))
        H[0][0] = 1.0
        H[1][1] = 1.0
        self._update(np.array([gps_x, gps_y]), H, self.R_gps)

    def update_imu(self, yaw):
        H = np.zeros((1, 5))
        H[0][2] = 1.0
        self._update(np.array([yaw]), H, self.R_imu)

    def update_radar(self, distance):
        """
        Radar gives forward distance — helps constrain
        velocity and detect wall proximity
        """
        if distance <= 0:
            return
        H = np.zeros((1, 5))
        H[0][3] = 1.0   # maps to vx
        # Use radar to constrain forward velocity
        # If obstacle close → vx should be low
        expected_vx = min(self.x[3], distance * 0.5)
        self._update(np.array([expected_vx]), H, self.R_radar)

    def update_ultrasonics(self, front, right, left):
        """
        Ultrasonics give boundary distances
        Help detect lake edges → constrain position
        """
        valid = []
        H_rows = []
        if front > 0 and front < 4.0:
            valid.append(front)
            h = np.zeros(5)
            h[3] = 1.0
            H_rows.append(h)
        if not valid:
            return
        H = np.array(H_rows)
        z = np.array(valid)
        R = self.R_ultrasonic[:len(valid), :len(valid)]
        self._update(z, H, R)

    def update_camera(self, dx, dy):
        """Visual odometry delta — camera optical flow"""
        if dx == 0.0 and dy == 0.0:
            return
        H = np.zeros((2, 5))
        H[0][3] = 1.0   # vx
        H[1][4] = 1.0   # vy
        self._update(np.array([dx, dy]), H, self.R_camera)


class LocalizationNode(Node):
    def __init__(self):
        super().__init__('asv_localization_node')

        self.declare_parameter('publish_hz',   20.0)
        self.declare_parameter('frame_id',     'map')
        self.declare_parameter('child_frame',  'base_link')

        self.pub_hz      = self.get_parameter('publish_hz').value
        self.frame_id    = self.get_parameter('frame_id').value
        self.child_frame = self.get_parameter('child_frame').value

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=10)

        # ── Subscribers ──────────────────────────────────────────
        self.create_subscription(NavSatFix, '/asv/gps/fix',          self._gps_cb,        sensor_qos)
        self.create_subscription(Imu,       '/asv/imu/data',         self._imu_cb,        sensor_qos)
        self.create_subscription(Float32,   '/asv/radar/distance',   self._radar_cb,      sensor_qos)
        self.create_subscription(Range,     '/asv/ultrasonic/front', self._front_us_cb,   sensor_qos)
        self.create_subscription(Range,     '/asv/ultrasonic/right', self._right_us_cb,   sensor_qos)
        self.create_subscription(Range,     '/asv/ultrasonic/left',  self._left_us_cb,    sensor_qos)
        self.create_subscription(Image,     '/asv/camera/image_raw', self._camera_cb,     sensor_qos)

        # ── Publishers ────────────────────────────────────────────
        self.pose_pub   = self.create_publisher(PoseStamped, '/asv/localization/pose',   10)
        self.odom_pub   = self.create_publisher(Odometry,    '/asv/localization/odom',   10)
        self.status_pub = self.create_publisher(String,      '/asv/localization/status', 10)

        # TF
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # EKF
        self.ekf = MultiSensorEKF()

        # GPS origin
        self.proj       = None
        self.origin_lat = None
        self.origin_lon = None

        # Timing
        self.last_time = None

        # Ultrasonic values
        self.us_front = -1.0
        self.us_right = -1.0
        self.us_left  = -1.0

        # Camera optical flow
        self.bridge    = CvBridge()
        self.prev_gray = None
        self.cam_dx    = 0.0
        self.cam_dy    = 0.0

        self.timer = self.create_timer(1.0 / self.pub_hz, self._publish)
        self.get_logger().info('Multi-sensor localization node started')

    # ── GPS callback ──────────────────────────────────────────────
    def _gps_cb(self, msg):
        if msg.status.status < 0:
            return
        if self.origin_lat is None:
            self.origin_lat = msg.latitude
            self.origin_lon = msg.longitude
            self.proj = Proj(
                proj='aeqd',
                lat_0=self.origin_lat,
                lon_0=self.origin_lon,
                datum='WGS84')
            self.ekf.initialized = True
            self.get_logger().info(
                f'Origin set: lat={self.origin_lat:.6f} lon={self.origin_lon:.6f}')
            return
        x, y = self.proj(msg.longitude, msg.latitude)
        self.ekf.update_gps(x, y)

    # ── IMU callback ──────────────────────────────────────────────
    def _imu_cb(self, msg):
        q = msg.orientation
        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self.ekf.update_imu(yaw)

        now = self.get_clock().now().nanoseconds * 1e-9
        if self.last_time is not None:
            dt = now - self.last_time
            self.ekf.predict(dt)
        self.last_time = now

    # ── Radar callback ────────────────────────────────────────────
    def _radar_cb(self, msg):
        self.ekf.update_radar(msg.data)

    # ── Ultrasonic callbacks ──────────────────────────────────────
    def _front_us_cb(self, msg):
        self.us_front = msg.range
        self.ekf.update_ultrasonics(
            self.us_front, self.us_right, self.us_left)

    def _right_us_cb(self, msg):
        self.us_right = msg.range

    def _left_us_cb(self, msg):
        self.us_left = msg.range

    # ── Camera optical flow callback ──────────────────────────────
    def _camera_cb(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            gray  = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            gray  = cv2.resize(gray, (320, 240))

            if self.prev_gray is not None:
                flow = cv2.calcOpticalFlowFarneback(
                    self.prev_gray, gray,
                    None, 0.5, 3, 15, 3, 5, 1.2, 0)
                self.cam_dx = float(np.mean(flow[..., 0])) * 0.01
                self.cam_dy = float(np.mean(flow[..., 1])) * 0.01
                self.ekf.update_camera(self.cam_dx, self.cam_dy)

            self.prev_gray = gray
        except Exception as e:
            self.get_logger().debug(f'Camera flow skip: {e}')

    # ── Publish ───────────────────────────────────────────────────
    def _publish(self):
        if not self.ekf.initialized:
            s = String()
            s.data = '[LOC] Waiting for GPS fix...'
            self.status_pub.publish(s)
            return

        now = self.get_clock().now().to_msg()
        x, y, yaw, vx, vy = self.ekf.x

        # PoseStamped
        pose = PoseStamped()
        pose.header.stamp    = now
        pose.header.frame_id = self.frame_id
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = 0.0
        pose.pose.orientation.z = math.sin(yaw / 2.0)
        pose.pose.orientation.w = math.cos(yaw / 2.0)
        self.pose_pub.publish(pose)

        # Odometry
        odom = Odometry()
        odom.header.stamp    = now
        odom.header.frame_id = self.frame_id
        odom.child_frame_id  = self.child_frame
        odom.pose.pose       = pose.pose
        odom.twist.twist.linear.x = vx
        odom.twist.twist.linear.y = vy
        self.odom_pub.publish(odom)

        # TF
        t = TransformStamped()
        t.header.stamp    = now
        t.header.frame_id = self.frame_id
        t.child_frame_id  = self.child_frame
        t.transform.translation.x = x
        t.transform.translation.y = y
        t.transform.translation.z = 0.0
        t.transform.rotation.z = math.sin(yaw / 2.0)
        t.transform.rotation.w = math.cos(yaw / 2.0)
        self.tf_broadcaster.sendTransform(t)

        # Status
        s = String()
        s.data = (
            f'[LOC] x={x:.2f}m y={y:.2f}m '
            f'hdg={math.degrees(yaw)%360:.1f}° '
            f'vx={vx:.2f} vy={vy:.2f} | '
            f'US F={self.us_front:.2f} '
            f'R={self.us_right:.2f} '
            f'L={self.us_left:.2f}'
        )
        self.status_pub.publish(s)


def main(args=None):
    rclpy.init(args=args)
    node = LocalizationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
