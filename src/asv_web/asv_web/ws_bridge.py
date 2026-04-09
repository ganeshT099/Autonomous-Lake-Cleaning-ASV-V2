import asyncio
import json
import websockets
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from threading import Thread

SERVER = "wss://asv-server.onrender.com/ws/client"


class GPSNode(Node):
    def __init__(self):
        super().__init__('asv_ws_bridge')

        self.lat = 0.0
        self.lon = 0.0

        self.create_subscription(
            NavSatFix,
            '/asv/gps/fix',
            self.callback,
            10
        )

    def callback(self, msg):
        self.lat = msg.latitude
        self.lon = msg.longitude


def ros_spin(node):
    rclpy.spin(node)


async def send_data(node):
    while True:
        try:
            async with websockets.connect(SERVER) as ws:
                print("Connected to cloud")

                while True:
                    data = {
                        "gps": {
                            "lat": node.lat,
                            "lon": node.lon
                        },
                        "status": "RUNNING"
                    }

                    await ws.send(json.dumps(data))
                    await asyncio.sleep(1)

        except Exception as e:
            print("Reconnect:", e)
            await asyncio.sleep(2)


def main():
    rclpy.init()
    node = GPSNode()

    Thread(target=ros_spin, args=(node,), daemon=True).start()

    asyncio.run(send_data(node))


if __name__ == "__main__":
    main()
