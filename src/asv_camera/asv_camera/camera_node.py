import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2


class CameraNode(Node):

    def __init__(self):
        super().__init__('asv_camera_node')

        self.bridge = CvBridge()

        self.cap = cv2.VideoCapture(0, cv2.CAP_V4L2)

        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        self.cap.set(cv2.CAP_PROP_FPS, 30)

        self.publisher = self.create_publisher(Image, '/camera/image_raw', 10)

        # TIMER LOOP (THIS IS IMPORTANT)
        self.timer = self.create_timer(0.03, self.publish_frame)

        self.get_logger().info("Camera node started")


    def publish_frame(self):
        ret, frame = self.cap.read()

        if not ret:
            self.get_logger().error("Frame capture failed")
            return

        msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        self.publisher.publish(msg)

        # Debug log
        self.get_logger().info("Publishing frame")


def main(args=None):
    rclpy.init(args=args)
    node = CameraNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
