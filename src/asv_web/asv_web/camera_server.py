#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage

from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

import threading
import time

from flask import Flask, Response

app = Flask(__name__)

latest_frame = None
lock = threading.Lock()


class CameraSubscriber(Node):
    def __init__(self):
        super().__init__('camera_stream_node')

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.last_time = time.time()

        self.subscription = self.create_subscription(
            CompressedImage,
            '/asv/camera/image_raw/compressed',
            self.listener_callback,
            qos
        )

        self.get_logger().info("Camera Subscriber Started")

    def listener_callback(self, msg):
        global latest_frame

        # Limit to ~10 Hz
        if time.time() - self.last_time < 0.1:
            return
        self.last_time = time.time()

        try:
            with lock:
                latest_frame = bytes(msg.data)
        except Exception as e:
            self.get_logger().error(f"Frame error: {e}")


def generate():
    global latest_frame

    while True:
        with lock:
            frame = latest_frame

        if frame is None:
            time.sleep(0.05)
            continue

        yield (
            b'--frame\r\n'
            b'Content-Type: image/jpeg\r\n\r\n' +
            frame +
            b'\r\n'
        )

        time.sleep(0.1)


@app.route('/video')
def video_feed():
    return Response(generate(),
                    mimetype='multipart/x-mixed-replace; boundary=frame')


@app.route('/')
def index():
    return '<html><body><img src="/video" width="640" height="480"></body></html>'


def main():
    rclpy.init()

    node = CameraSubscriber()

    ros_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    ros_thread.start()

    print("Camera stream: http://192.168.0.50:5000/video")

    app.run(host='0.0.0.0', port=5000, threaded=True)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
