import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from flask import Flask, Response

from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

import cv2
import threading
import time

app = Flask(__name__)

frame = None


class CameraServer(Node):
    def __init__(self):
        super().__init__('camera_server')

        self.bridge = CvBridge()

        # 🔥 MATCH CAMERA NODE QoS
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.create_subscription(
            Image,
            '/asv/camera/image_raw',
            self.callback,
            qos
        )

    def callback(self, msg):
        global frame
        try:
            img = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            frame = img
        except:
            pass


def generate():
    global frame
    while True:
        if frame is not None:
            _, buffer = cv2.imencode('.jpg', frame, [int(cv2.IMWRITE_JPEG_QUALITY), 60])

            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' +
                   buffer.tobytes() + b'\r\n')

        time.sleep(0.05)


@app.route('/video')
def video():
    return Response(generate(),
                    mimetype='multipart/x-mixed-replace; boundary=frame')


def ros_spin(node):
    rclpy.spin(node)


def main():
    rclpy.init()
    node = CameraServer()

    threading.Thread(target=ros_spin, args=(node,), daemon=True).start()

    print("🎥 Camera → http://192.168.0.50:5000/video")

    app.run(host='0.0.0.0', port=5000)


if __name__ == '__main__':
    main()
