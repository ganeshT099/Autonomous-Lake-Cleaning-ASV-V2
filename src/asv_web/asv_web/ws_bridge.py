#!/usr/bin/env python3
"""ASV WebSocket Bridge  v7 — on-demand single camera stream"""

import asyncio
import base64
import json
import queue
import threading
import time
import websockets
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, qos_profile_sensor_data
from sensor_msgs.msg import NavSatFix, CompressedImage
from geometry_msgs.msg import Twist

SERVER = "ws://localhost:8080/ws/device"

_raw_latest:  bytes = None
_seg_latest:  bytes = None
_yolo_latest: bytes = None
_raw_lock  = threading.Lock()
_seg_lock  = threading.Lock()
_yolo_lock = threading.Lock()

_active_stream = "raw"          # which stream browser wants
_stream_lock   = threading.Lock()

_seg_queue  = queue.Queue(maxsize=1)
_yolo_queue = queue.Queue(maxsize=1)
_raw_queue  = queue.Queue(maxsize=1)

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
        self.create_subscription(CompressedImage, "/asv/camera/image_raw/compressed", self._cam_cb, qos_be)
        self.create_subscription(Twist, "/cmd_vel", self._cmdvel_cb, 10)
        self._cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.get_logger().info("ASV Bridge Node v7 ready (on-demand camera)")

    def _gps_cb(self, msg):
        self.lat = msg.latitude
        self.lon = msg.longitude

    def _cam_cb(self, msg):
        now = time.time()
        if now - self._last_frame_ts < 0.33:   # ~3 Hz cap
            return
        self._last_frame_ts = now
        raw = bytes(msg.data)
        with _stream_lock:
            active = _active_stream
        # Only push to active stream queue
        q_map = {"raw": _raw_queue, "seg": _seg_queue, "yolo": _yolo_queue}
        try:
            q_map[active].put_nowait(raw)
        except (queue.Full, KeyError):
            pass

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


def _process_raw_frames():
    global _raw_latest
    import cv2, numpy as np
    print("[raw] Raw thread ready.", flush=True)
    while True:
        try:
            raw = _raw_queue.get(timeout=1.0)
        except queue.Empty:
            continue
        try:
            arr   = np.frombuffer(raw, np.uint8)
            frame = cv2.imdecode(arr, cv2.IMREAD_COLOR)
            if frame is None:
                continue
            frame = cv2.resize(frame, (640, 360))
            cv2.putText(frame, "RAW", (8, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (160,160,160), 1, cv2.LINE_AA)
            _, enc = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 75])
            with _raw_lock:
                _raw_latest = enc.tobytes()
        except Exception as e:
            print(f"[raw] {e}", flush=True)


def _process_seg_frames():
    global _seg_latest
    import sys, os
    _pkg_dir = os.path.dirname(os.path.abspath(__file__))
    if _pkg_dir not in sys.path:
        sys.path.insert(0, _pkg_dir)
    print("[seg] Loading ShoreDetector...", flush=True)
    try:
        from shore_detector import ShoreDetector
        detector = ShoreDetector()
        print("[seg] ShoreDetector ready.", flush=True)
    except Exception as e:
        print(f"[seg] Load failed: {e}", flush=True)
        detector = None
    while True:
        try:
            raw = _seg_queue.get(timeout=1.0)
        except queue.Empty:
            continue
        try:
            processed = detector.process_jpeg(raw) if detector else raw
        except Exception as e:
            print(f"[seg] {e}", flush=True)
            processed = raw
        with _seg_lock:
            _seg_latest = processed


def _process_yolo_frames():
    global _yolo_latest
    import cv2, numpy as np
    print("[yolo] Loading YOLO11n...", flush=True)
    model = None
    try:
        from ultralytics import YOLO
        import os
        search = [
            os.path.expanduser("~/yolo11n.pt"),
            os.path.expanduser("~/models/yolo11n.pt"),
            "yolo11n.pt",
        ]
        model_path = next((p for p in search if os.path.exists(p)), "yolo11n.pt")
        model = YOLO(model_path)
        # warmup with numpy array (not bytes)
        dummy = np.zeros((360, 640, 3), np.uint8)
        model.predict(dummy, verbose=False, imgsz=640)
        print(f"[yolo] Ready — {model_path}", flush=True)
    except Exception as e:
        print(f"[yolo] Load failed: {e}", flush=True)
        model = None
    while True:
        try:
            raw = _yolo_queue.get(timeout=1.0)
        except queue.Empty:
            continue
        try:
            arr   = np.frombuffer(raw, np.uint8)
            frame = cv2.imdecode(arr, cv2.IMREAD_COLOR)
            if frame is None:
                continue
            frame = cv2.resize(frame, (640, 360))
            if model is not None:
                results   = model.predict(frame, verbose=False, imgsz=640)[0]
                annotated = results.plot()
            else:
                annotated = frame.copy()
                cv2.putText(annotated, "YOLO — model not loaded", (8, 30),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,0,255), 2, cv2.LINE_AA)
            _, enc = cv2.imencode(".jpg", annotated, [cv2.IMWRITE_JPEG_QUALITY, 75])
            with _yolo_lock:
                _yolo_latest = enc.tobytes()
        except Exception as e:
            print(f"[yolo] {e}", flush=True)


async def _send_telemetry(ws):
    while True:
        payload = json.dumps({
            "type":    "telemetry",
            "gps":     {"lat": _node.lat, "lon": _node.lon},
            "status":  _node.status,
            "cmd_vel": {"linear": round(_node.cmd_linear, 4), "angular": round(_node.cmd_angular, 4)},
        })
        await ws.send(payload)
        await asyncio.sleep(0.2)


async def _send_active_camera(ws):
    """Send only the active stream at ~2 Hz."""
    lock_map = {"raw": _raw_lock, "seg": _seg_lock, "yolo": _yolo_lock}
    def get_frame(stream):
        global _raw_latest, _seg_latest, _yolo_latest
        return {"raw": _raw_latest, "seg": _seg_latest, "yolo": _yolo_latest}.get(stream)

    while True:
        with _stream_lock:
            stream = _active_stream
        lock = lock_map.get(stream, _raw_lock)
        with lock:
            frame = get_frame(stream)
        if frame:
            msg_type = f"camera_{stream}"
            b64 = base64.b64encode(frame).decode("ascii")
            await ws.send(json.dumps({"type": msg_type, "data": b64}))
        await asyncio.sleep(0.5)   # 2 Hz — easy on Render


async def _send_heartbeat(ws):
    while True:
        await asyncio.sleep(4)
        await ws.send(json.dumps({"type": "ping"}))


async def _recv_commands(ws):
    global _active_stream
    async for raw in ws:
        if not isinstance(raw, str):
            continue
        try:
            data = json.loads(raw)
            t = data.get("type")
            if t == "pong":
                pass
            elif t == "camera_select":
                stream = data.get("stream", "raw")
                if stream in ("raw", "seg", "yolo"):
                    with _stream_lock:
                        _active_stream = stream
                    print(f"[bridge] Camera stream → {stream}", flush=True)
            elif t == "joy":
                lin, ang = compute_cmd_vel(data.get("axes", []), data.get("buttons", []))
                _node.publish_cmd_vel(lin, ang)
        except Exception as e:
            print(f"[recv] {e}", flush=True)


async def bridge():
    while True:
        try:
            async with websockets.connect(SERVER, ping_interval=None, open_timeout=15) as ws:
                print("[bridge] Connected to relay", flush=True)
                await asyncio.gather(
                    _send_telemetry(ws),
                    _send_active_camera(ws),
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
    threading.Thread(target=rclpy.spin,            args=(_node,), daemon=True).start()
    threading.Thread(target=_process_raw_frames,   daemon=True).start()
    threading.Thread(target=_process_seg_frames,   daemon=True).start()
    threading.Thread(target=_process_yolo_frames,  daemon=True).start()
    try:
        asyncio.run(bridge())
    except KeyboardInterrupt:
        pass
    _node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
