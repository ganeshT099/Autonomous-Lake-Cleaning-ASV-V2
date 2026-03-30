import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image, CompressedImage
from geometry_msgs.msg import Point

from cv_bridge import CvBridge
import cv2
import numpy as np

import os
from ament_index_python.packages import get_package_share_directory
from ultralytics import YOLO


class TrashDetection(Node):
    def __init__(self):
        super().__init__('trash_detection_node')

        # Model path
        package_path = get_package_share_directory('asv_trash_detection')
        model_path = os.path.join(package_path, 'models', 'trash_seg.pt')

        self.get_logger().info(f"Model: {model_path}")

        self.model = YOLO(model_path)

        self.bridge = CvBridge()

        # Subscribe camera
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )

        # Publish centroid
        self.centroid_pub = self.create_publisher(
            Point,
            '/trash_centroid',
            10
        )

        # 🔥 Publish compressed stream
        self.stream_pub = self.create_publisher(
            CompressedImage,
            '/detection/compressed',
            10
        )

        # smoothing
        self.prev_x = 0
        self.prev_y = 0

    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')

        # Faster inference
        results = self.model(frame, imgsz=320, conf=0.3)

        if results and len(results[0].boxes) > 0:
            box = results[0].boxes[0]

            x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
            conf = float(box.conf[0])

            if conf < 0.4:
                return

            cx = int((x1 + x2) / 2)
            cy = int((y1 + y2) / 2)

            # smoothing
            cx = int(0.7 * self.prev_x + 0.3 * cx)
            cy = int(0.7 * self.prev_y + 0.3 * cy)

            self.prev_x = cx
            self.prev_y = cy

            # publish centroid
            point = Point()
            point.x = float(cx)
            point.y = float(cy)
            point.z = 0.0
            self.centroid_pub.publish(point)

        # draw detections
        annotated = results[0].plot()

        # 🔥 convert to compressed
        _, buffer = cv2.imencode('.jpg', annotated)
        msg_out = CompressedImage()
        msg_out.format = "jpeg"
        msg_out.data = buffer.tobytes()

        self.stream_pub.publish(msg_out)


def main(args=None):
    rclpy.init(args=args)
    node = TrashDetection()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
