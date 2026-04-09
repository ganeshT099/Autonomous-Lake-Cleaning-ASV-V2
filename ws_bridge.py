import asyncio
import websockets
import json
import base64

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from sensor_msgs.msg import NavSatFix, Image
from cv_bridge import CvBridge
import cv2

SERVER = "wss://asv-server.onrender.com/ws/device"

class ASVBridge(Node):
    def __init__(self):
        super().__init__('asv_bridge')

        qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT
        )

        self.bridge = CvBridge()
        self.latest_gps = None
        self.latest_frame = None

        # GPS
        self.create_subscription(
            NavSatFix,
            '/asv/fix',
            self.gps_callback,
            qos
        )

        # CAMERA
        self.create_subscription(
            Image,
            '/asv/camera/image_raw',
            self.cam_callback,
            qos
        )

    def gps_callback(self, msg):
        self.latest_gps = {
            "lat": msg.latitude,
            "lon": msg.longitude
        }

    def cam_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            # compress to JPEG
            _, buffer = cv2.imencode('.jpg', frame, [int(cv2.IMWRITE_JPEG_QUALITY), 50])

            # convert to base64
            self.latest_frame = base64.b64encode(buffer).decode('utf-8')

        except Exception as e:
            print("Camera error:", e)


async def send_data(node):
    while True:
        try:
            print("Connecting...")
            async with websockets.connect(SERVER) as ws:
                print("Connected to server")

                while True:
                    rclpy.spin_once(node, timeout_sec=0.1)

                    payload = {
                        "status": "RUNNING",
                        "gps": node.latest_gps,
                        "image": node.latest_frame
                    }

                    await ws.send(json.dumps(payload))
                    await asyncio.sleep(0.2)

        except Exception as e:
            print("Reconnect:", e)
            await asyncio.sleep(3)


def main():
    rclpy.init()
    node = ASVBridge()
    asyncio.run(send_data(node))


if __name__ == "__main__":
    main()
