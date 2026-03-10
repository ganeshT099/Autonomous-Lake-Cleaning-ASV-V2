#!/usr/bin/env python3
"""
ASV Camera Node — Microsoft LifeCam (USB V4L2)
Publishes:
  /asv/camera/image_raw     (sensor_msgs/Image)
  /asv/camera/camera_info   (sensor_msgs/CameraInfo)
  /asv/camera/status        (std_msgs/String)
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import threading
import time

class CameraNode(Node):
    def __init__(self):
        super().__init__('asv_camera_node')

        self.declare_parameter('device_id',  0)
        self.declare_parameter('width',      1280)
        self.declare_parameter('height',     720)
        self.declare_parameter('fps',        30)
        self.declare_parameter('frame_id',   'camera_link')
        self.declare_parameter('publish_hz', 30.0)

        self.device_id = self.get_parameter('device_id').value
        self.width     = self.get_parameter('width').value
        self.height    = self.get_parameter('height').value
        self.fps       = self.get_parameter('fps').value
        self.frame_id  = self.get_parameter('frame_id').value
        self.pub_hz    = self.get_parameter('publish_hz').value

        img_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=5)

        self.img_pub    = self.create_publisher(Image,      '/asv/camera/image_raw',   img_qos)
        self.info_pub   = self.create_publisher(CameraInfo, '/asv/camera/camera_info', img_qos)
        self.status_pub = self.create_publisher(String,     '/asv/camera/status',      10)

        self.bridge     = CvBridge()
        self.frame      = None
        self.frame_lock = threading.Lock()
        self.running    = True

        self._open_camera()

        self.grab_thread = threading.Thread(target=self._grab_loop, daemon=True)
        self.grab_thread.start()

        self.timer = self.create_timer(1.0 / self.pub_hz, self._publish)
        self.get_logger().info(
            f'Camera node started | /dev/video{self.device_id} | {self.width}x{self.height}@{self.fps}fps')

    def _open_camera(self):
        while True:
            cap = cv2.VideoCapture(self.device_id, cv2.CAP_V4L2)
            if cap.isOpened():
                cap.set(cv2.CAP_PROP_FRAME_WIDTH,  self.width)
                cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
                cap.set(cv2.CAP_PROP_FPS,          self.fps)
                self.cap = cap
                self.get_logger().info(f'Camera opened: /dev/video{self.device_id}')
                return
            self.get_logger().error(f'Cannot open /dev/video{self.device_id} retrying in 3s')
            time.sleep(3.0)

    def _grab_loop(self):
        while self.running:
            if self.cap and self.cap.isOpened():
                ret, frame = self.cap.read()
                if ret:
                    with self.frame_lock:
                        self.frame = frame
                else:
                    self.get_logger().warn('Camera read failed — reopening')
                    self.cap.release()
                    self._open_camera()
            else:
                time.sleep(0.1)

    def _camera_info(self, stamp):
        info = CameraInfo()
        info.header.stamp    = stamp
        info.header.frame_id = self.frame_id
        info.width  = self.width
        info.height = self.height
        info.distortion_model = 'plumb_bob'
        fx = fy = float(self.width)
        cx = self.width  / 2.0
        cy = self.height / 2.0
        info.k = [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]
        info.d = [0.0, 0.0, 0.0, 0.0, 0.0]
        info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        info.p = [fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0]
        return info

    def _publish(self):
        with self.frame_lock:
            frame = self.frame.copy() if self.frame is not None else None

        if frame is None:
            s = String()
            s.data = '[CAMERA] Waiting for frame...'
            self.status_pub.publish(s)
            return

        now = self.get_clock().now().to_msg()

        img_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        img_msg.header.stamp    = now
        img_msg.header.frame_id = self.frame_id
        self.img_pub.publish(img_msg)
        self.info_pub.publish(self._camera_info(now))

        h, w = frame.shape[:2]
        s = String()
        s.data = f'[CAMERA] Publishing {w}x{h} | /dev/video{self.device_id}'
        self.status_pub.publish(s)

    def destroy_node(self):
        self.running = False
        if self.cap:
            self.cap.release()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = CameraNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
