#!/usr/bin/env python3
"""
ASV WebSocket Bridge  v4
  - Sends ping every 4s to keep Render proxy alive (gets pong back)
  - Camera as base64 JSON text
  - GPS + cmd_vel telemetry at 10 Hz
  - Publishes /cmd_vel from joystick input
"""

import asyncio
import base64
import json
import threading
import time
import websockets
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, qos_profile_sensor_data
from sensor_msgs.msg import NavSatFix, CompressedImage
from geometry_msgs.msg import Twist

SERVER = "ws://localhost:8080/ws/device"

_latest_frame: bytes = None
_frame_lock = threading.Lock()
_node = None


def _expo(x):
    return x * x * (1.0 if x >= 0 else -1.0)


def compute_cmd_vel(axes, buttons):
    rb    = buttons[5] if len(buttons) > 5 else 0
    rt    = float(buttons[7]) if len(buttons) > 7 else 0.0
    lt    = float(buttons[6]) if len(buttons) > 6 else 0.0
    steer = float(axes[2])    if len(axes) > 2   else 0.0
    boost = buttons[4]        if len(buttons) > 4 else 0

    if not rb:
        return 0.0, 0.0

    linear_raw = rt - lt
    if linear_raw > 0:
        linear = 0.6 + 0.4 * linear_raw
    elif linear_raw < 0:
        linear = -0.6 + 0.4 * linear_raw
    else:
        linear = 0.0

    linear = _expo(linear)
    if boost:
        linear = max(-1.0, min(1.0, linear * 1.15))
    steer = 0.0 if abs(steer) < 0.1 else steer
    return linear, steer


class ASVBridgeNode(Node):
    def __init__(self):
        super().__init__("asv_ws_bridge")
        self.lat = 0.0
        self.lon = 0.0
        self.status = "RUNNING"
        self.cmd_linear  = 0.0
        self.cmd_angular = 0.0
        self._last_frame_ts = 0.0

        qos_be = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.create_subscription(NavSatFix, "/asv/gps/fix", self._gps_cb, qos_profile_sensor_data)
        self.create_subscription(
            CompressedImage,
            "/asv/camera/image_raw/compressed",
            self._cam_cb, qos_be,
        )
        self.create_subscription(Twist, "/cmd_vel", self._cmdvel_cb, 10)
        self._cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.get_logger().info("ASV Bridge Node v4 ready")

    def _gps_cb(self, msg):
        self.lat = msg.latitude
        self.lon = msg.longitude

    def _cam_cb(self, msg):
        global _latest_frame
        now = time.time()
        if now - self._last_frame_ts < 0.2:   # 5 Hz (reduced from 10 to save bandwidth)
            return
        self._last_frame_ts = now
        with _frame_lock:
            _latest_frame = bytes(msg.data)

    def _cmdvel_cb(self, msg):
        self.cmd_linear  = msg.linear.x
        self.cmd_angular = msg.angular.z

    def publish_cmd_vel(self, linear, angular):
        msg = Twist()
        msg.linear.x  = linear
        msg.angular.z = angular
        self._cmd_pub.publish(msg)
        self.cmd_linear  = linear
        self.cmd_angular = angular


async def _send_telemetry(ws):
    while True:
        payload = json.dumps({
            "type": "telemetry",
            "gps":    {"lat": _node.lat, "lon": _node.lon},
            "status": _node.status,
            "cmd_vel": {
                "linear":  round(_node.cmd_linear,  4),
                "angular": round(_node.cmd_angular, 4),
            },
        })
        await ws.send(payload)
        await asyncio.sleep(0.2)   # 5 Hz telemetry


async def _send_camera(ws):
    global _latest_frame
    while True:
        with _frame_lock:
            frame = _latest_frame

        if frame:
            b64 = base64.b64encode(frame).decode("ascii")
            await ws.send(json.dumps({"type": "camera", "data": b64}))

        await asyncio.sleep(0.2)   # 5 Hz camera


async def _send_heartbeat(ws):
    """Ping relay every 4s so Render proxy sees bidirectional traffic."""
    while True:
        await asyncio.sleep(4)
        await ws.send(json.dumps({"type": "ping"}))


async def _recv_commands(ws):
    async for raw in ws:
        if not isinstance(raw, str):
            continue
        try:
            data = json.loads(raw)
            if data.get("type") == "pong":
                pass   # heartbeat ack — good
            elif data.get("type") == "joy":
                lin, ang = compute_cmd_vel(
                    data.get("axes",    []),
                    data.get("buttons", []),
                )
                _node.publish_cmd_vel(lin, ang)
        except Exception as e:
            print(f"[recv] {e}", flush=True)


async def bridge():
    while True:
        try:
            async with websockets.connect(
                SERVER,
                ping_interval=None,   # disable library pings; we do app-level
                open_timeout=15,
            ) as ws:
                print("[bridge] Connected to relay", flush=True)
                await asyncio.gather(
                    _send_telemetry(ws),
                    _send_camera(ws),
                    _send_heartbeat(ws),
                    _recv_commands(ws),
                )
        except Exception as e:
            print(f"[bridge] Reconnecting — {e}", flush=True)
            await asyncio.sleep(3)


def main():
    global _node
    rclpy.init()
    _node = ASVBridgeNode()
    threading.Thread(target=rclpy.spin, args=(_node,), daemon=True).start()
    try:
        asyncio.run(bridge())
    except KeyboardInterrupt:
        pass
    _node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
