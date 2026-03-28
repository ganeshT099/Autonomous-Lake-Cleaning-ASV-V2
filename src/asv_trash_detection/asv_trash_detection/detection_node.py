import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray
from cv_bridge import CvBridge

import cv2
import os
from ultralytics import YOLO


class TrashDetection(Node):

    def __init__(self):
        super().__init__('trash_detection_node')

        self.bridge = CvBridge()

        # Load model
        model_path = os.path.join(
            os.path.dirname(__file__),
            'models',
            'trash_seg.pt'
        )

        self.get_logger().info(f"Loading model from: {model_path}")
        self.model = YOLO(model_path)

        # Subscribe to camera
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',   # make sure this matches your camera topic
            self.image_callback,
            10
        )

        # Publish centroid
        self.publisher = self.create_publisher(
            Float32MultiArray,
            '/trash_centroid',
            10
        )

        # ✅ Create window ONCE (IMPORTANT)
        cv2.namedWindow("Trash Detection", cv2.WINDOW_NORMAL)

        self.get_logger().info("🚀 Trash Detection Node Started")


    def image_callback(self, msg):

        # Convert ROS → OpenCV
        frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')

        results = self.model(frame)

        detected = False
        cx, cy = -1, -1

        for r in results:
            if r.boxes is not None and len(r.boxes) > 0:

                detected = True

                for box in r.boxes.xyxy:
                    x1, y1, x2, y2 = map(int, box)

                    # Draw box
                    cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)

                    # Centroid
                    cx = int((x1 + x2) / 2)
                    cy = int((y1 + y2) / 2)

                    cv2.circle(frame, (cx, cy), 5, (0, 0, 255), -1)

        # Publish centroid
        msg_out = Float32MultiArray()

        if detected:
            msg_out.data = [float(cx), float(cy)]
        else:
            msg_out.data = [-1.0, -1.0]

        self.publisher.publish(msg_out)

        # ✅ Show frame
        cv2.imshow("Trash Detection", frame)
        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)

    node = TrashDetection()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()

    # ✅ Clean close window
    cv2.destroyAllWindows()

    rclpy.shutdown()
