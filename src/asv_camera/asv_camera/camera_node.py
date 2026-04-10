import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

import cv2


class CameraNode(Node):

    def __init__(self):
        super().__init__('asv_camera_node')

        self.bridge = CvBridge()

        # Open camera
        self.cap = cv2.VideoCapture(0, cv2.CAP_V4L2)

        # 🔥 SAFE SETTINGS
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        self.cap.set(cv2.CAP_PROP_FPS, 15)

        # 🔥 QoS FIX
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # 🔥 CORRECT TOPIC
        self.publisher = self.create_publisher(
            Image,
            '/asv/camera/image_raw',
            qos
        )

        # 🔥 STABLE FPS (~10Hz)
        self.timer = self.create_timer(0.1, self.publish_frame)

        self.get_logger().info("Camera node started (stable)")


    def publish_frame(self):
        ret, frame = self.cap.read()
        if not ret:
            return

        msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = CameraNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.cap.release()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
