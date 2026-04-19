"""
Autonomous Mission Pilot — Jetson
Pure pursuit + Stanley CTE controller, geofence, correct topics.

Topics:
  SUB: /asv/gps/fix       (NavSatFix, BEST_EFFORT)
  SUB: /asv/imu/data      (Imu, BEST_EFFORT) — calibrated quaternion from imu_node
  PUB: /cmd_vel           (Twist) — what asv_thruster_node listens to
"""

import asyncio
import json
import math
import threading

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import NavSatFix, Imu
from geometry_msgs.msg import Twist
import websockets

RELAY_URL = "wss://asv-server-1.onrender.com/ws/auto_pilot"

# ── Geometry helpers ──────────────────────────────────────────────────────────
DEG2RAD = math.pi / 180
RAD2DEG = 180 / math.pi
EARTH_R  = 6_371_000.0

def haversine(lat1, lon1, lat2, lon2):
    dlat = (lat2 - lat1) * DEG2RAD
    dlon = (lon2 - lon1) * DEG2RAD
    a = math.sin(dlat/2)**2 + math.cos(lat1*DEG2RAD)*math.cos(lat2*DEG2RAD)*math.sin(dlon/2)**2
    return EARTH_R * 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))

def bearing_to(lat1, lon1, lat2, lon2):
    dlon = (lon2 - lon1) * DEG2RAD
    y = math.sin(dlon) * math.cos(lat2 * DEG2RAD)
    x = (math.cos(lat1*DEG2RAD)*math.sin(lat2*DEG2RAD)
         - math.sin(lat1*DEG2RAD)*math.cos(lat2*DEG2RAD)*math.cos(dlon))
    return math.atan2(y, x)

def wrap_angle(a):
    return ((a + math.pi) % (2 * math.pi) + 2 * math.pi) % (2 * math.pi) - math.pi

def cross_track_error(lat, lon, lat1, lon1, lat2, lon2):
    d13 = haversine(lat1, lon1, lat, lon)
    b13 = bearing_to(lat1, lon1, lat, lon)
    b12 = bearing_to(lat1, lon1, lat2, lon2)
    return EARTH_R * math.asin(
        max(-1.0, min(1.0, math.sin(d13/EARTH_R) * math.sin(b13-b12)))
    )

def point_in_polygon(lat, lon, polygon):
    """Ray-casting test. polygon = [[lat, lon], ...]"""
    inside = False
    j = len(polygon) - 1
    for i in range(len(polygon)):
        xi, yi = polygon[i]
        xj, yj = polygon[j]
        if ((yi > lon) != (yj > lon)) and (lat < (xj-xi)*(lon-yi)/(yj-yi+1e-12)+xi):
            inside = not inside
        j = i
    return inside


# ── ROS 2 node ────────────────────────────────────────────────────────────────
class AutonomousNode(Node):
    def __init__(self):
        super().__init__("autonomous_pilot")

        sensor_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )

        # ── Subscribers (fixed topics) ────────────────────────────────
        self.sub_gps = self.create_subscription(
            NavSatFix, "/asv/gps/fix", self._on_gps, sensor_qos)
        self.sub_imu = self.create_subscription(
            Imu, "/asv/imu/data", self._on_imu, sensor_qos)   # FIXED: was /asv/imu

        # ── Publisher (fixed topic: what asv_thruster_node subscribes) ─
        self.pub_cmd = self.create_publisher(Twist, "/cmd_vel", 10)   # FIXED: was /asv/cmd_vel

        # ── Sensor state ──────────────────────────────────────────────
        self.lat     = 0.0
        self.lon     = 0.0
        self.heading = 0.0   # radians, from IMU quaternion (bias-corrected)
        self.gps_ok  = False

        # ── Mission state ─────────────────────────────────────────────
        self.waypoints   = []       # [[lat, lon], ...]
        self.boundary    = []       # [[lat, lon], ...] geofence polygon
        self.wp_idx      = 0
        self.running     = False
        self.config      = {}
        self.cte         = 0.0
        self.cte_history = []
        self.lock        = threading.Lock()
        self.on_status   = None

        # Control loop at 10 Hz
        self.create_timer(0.1, self._control_loop)
        self.get_logger().info("AutonomousNode ready — pure pursuit + Stanley + geofence")

    # ── Sensor callbacks ──────────────────────────────────────────────
    def _on_gps(self, msg: NavSatFix):
        if math.isnan(msg.latitude) or math.isnan(msg.longitude):
            return
        if msg.status.status < 0:
            return
        self.lat    = msg.latitude
        self.lon    = msg.longitude
        self.gps_ok = True

    def _on_imu(self, msg: Imu):
        # Use calibrated quaternion yaw from imu_node (not raw gz integration)
        # imu_node stores: orientation.z = sin(yaw/2), orientation.w = cos(yaw/2)
        self.heading = 2.0 * math.atan2(msg.orientation.z, msg.orientation.w)

    # ── Mission control ───────────────────────────────────────────────
    def start_mission(self, waypoints, boundary, config):
        with self.lock:
            self.waypoints   = waypoints
            self.boundary    = boundary
            self.wp_idx      = 0
            self.running     = True
            self.config      = config
            self.cte         = 0.0
            self.cte_history = []
        self.get_logger().info(
            f"Mission started: {len(waypoints)} wps, {len(boundary)}-pt geofence")

    def stop_mission(self):
        with self.lock:
            self.running = False
        self._publish_stop()
        self.get_logger().info("Mission stopped")

    def _publish_stop(self):
        self.pub_cmd.publish(Twist())

    # ── Control loop ──────────────────────────────────────────────────
    def _control_loop(self):
        with self.lock:
            if not self.running or not self.gps_ok or not self.waypoints:
                return
            wp_idx    = self.wp_idx
            waypoints = self.waypoints
            boundary  = self.boundary
            config    = self.config

        # ── Geofence check ────────────────────────────────────────────
        if boundary and not point_in_polygon(self.lat, self.lon, boundary):
            self._emergency_stop(
                f"GEOFENCE BREACH at {self.lat:.6f},{self.lon:.6f} — stopping")
            return

        max_speed = float(config.get("speed",     0.5))
        lookahead = float(config.get("lookahead", 8.0))
        TURN_GAIN = 2.0
        K_CTE     = 0.4

        if wp_idx >= len(waypoints):
            self._finish_mission()
            return

        # ── Projection-based waypoint advancement ─────────────────────
        while wp_idx < len(waypoints) - 1:
            A = waypoints[wp_idx - 1] if wp_idx > 0 else waypoints[wp_idx]
            B = waypoints[wp_idx]
            dLat, dLon = B[0]-A[0], B[1]-A[1]
            len2 = dLat*dLat + dLon*dLon
            t = ((self.lat-A[0])*dLat + (self.lon-A[1])*dLon) / len2 if len2 > 0 else 0
            d = haversine(self.lat, self.lon, B[0], B[1])
            if t >= 1.0 or d < 1.5:
                wp_idx += 1
            else:
                break
        with self.lock:
            self.wp_idx = wp_idx

        if wp_idx >= len(waypoints):
            self._finish_mission()
            return

        # ── Pure pursuit: find lookahead point on path ────────────────
        remaining  = lookahead
        li         = wp_idx
        target_lat = waypoints[li][0]
        target_lon = waypoints[li][1]
        while li < len(waypoints) - 1:
            seg = haversine(waypoints[li][0], waypoints[li][1],
                            waypoints[li+1][0], waypoints[li+1][1])
            if seg >= remaining:
                t = remaining / max(seg, 1e-9)
                target_lat = waypoints[li][0] + t*(waypoints[li+1][0]-waypoints[li][0])
                target_lon = waypoints[li][1] + t*(waypoints[li+1][1]-waypoints[li][1])
                break
            remaining -= seg
            li += 1
            if li == len(waypoints) - 1:
                target_lat = waypoints[li][0]
                target_lon = waypoints[li][1]

        # ── Stanley controller ─────────────────────────────────────────
        desired_bearing = bearing_to(self.lat, self.lon, target_lat, target_lon)
        # CTE correction: steer toward path (Stanley term)
        cte_term      = math.atan2(K_CTE * self.cte, max_speed + 0.1)
        heading_error = wrap_angle(desired_bearing - self.heading - cte_term)
        angular_cmd   = max(-1.0, min(1.0, heading_error * TURN_GAIN))
        linear_cmd    = max_speed   # 0.0–1.0 normalized

        cmd = Twist()
        cmd.linear.x  = linear_cmd
        cmd.angular.z = angular_cmd
        self.pub_cmd.publish(cmd)

        # ── CTE for next tick ─────────────────────────────────────────
        if wp_idx > 0:
            p = waypoints[wp_idx - 1]
            self.cte = cross_track_error(
                self.lat, self.lon,
                p[0], p[1],
                waypoints[wp_idx][0], waypoints[wp_idx][1])
        self.cte_history.append(abs(self.cte))
        if len(self.cte_history) > 200:
            self.cte_history.pop(0)

        cte_avg = sum(self.cte_history) / len(self.cte_history)
        cte_max = max(self.cte_history)
        dist    = haversine(self.lat, self.lon, waypoints[wp_idx][0], waypoints[wp_idx][1])
        pct     = min(100.0, wp_idx / len(waypoints) * 100.0)

        status = {
            "type":          "autonomous_status",
            "status":        "FOLLOWING",
            "wp_idx":        wp_idx,
            "wp_total":      len(waypoints),
            "cte":           round(self.cte,  3),
            "cte_avg":       round(cte_avg,   3),
            "cte_max":       round(cte_max,   3),
            "heading_error": round(heading_error * RAD2DEG, 2),
            "angular_cmd":   round(angular_cmd, 3),
            "linear_cmd":    round(linear_cmd,  3),
            "heading":       round(self.heading * RAD2DEG % 360, 1),
            "dist_to_next":  round(dist, 1),
            "pct":           round(pct,  1),
            "lat":           self.lat,
            "lon":           self.lon,
        }
        if abs(self.cte) > 0.8:
            status["log"]   = f"CTE {self.cte:+.2f}m  ω={angular_cmd:+.2f}"
            status["level"] = "warn"
        elif wp_idx % 10 == 0:
            status["log"]   = f"WP {wp_idx}/{len(waypoints)}  {dist:.0f}m  hdg {self.heading*RAD2DEG%360:.1f}°"
            status["level"] = "info"

        if self.on_status:
            self.on_status(status)

    def _emergency_stop(self, reason):
        with self.lock:
            self.running = False
        self._publish_stop()
        if self.on_status:
            self.on_status({
                "type": "autonomous_status", "status": "ABORTED",
                "log": reason, "level": "error",
            })
        self.get_logger().error(reason)

    def _finish_mission(self):
        with self.lock:
            self.running = False
        self._publish_stop()
        if self.on_status:
            self.on_status({
                "type": "autonomous_status", "status": "COMPLETE",
                "pct": 100.0,
                "log": "MISSION COMPLETE ✓  All waypoints reached",
                "level": "success",
            })
        self.get_logger().info("Mission complete")


# ── WebSocket relay loop ──────────────────────────────────────────────────────
async def ws_loop(node: AutonomousNode):
    status_queue: asyncio.Queue = asyncio.Queue()

    async def _inner():
        loop = asyncio.get_running_loop()

        def on_status(msg):
            loop.call_soon_threadsafe(status_queue.put_nowait, msg)

        node.on_status = on_status

        while True:
            try:
                async with websockets.connect(RELAY_URL, ping_interval=20) as ws:
                    print("[auto_pilot] Connected to relay", flush=True)

                    async def send_status():
                        while True:
                            try:
                                msg = await asyncio.wait_for(status_queue.get(), timeout=0.5)
                                await ws.send(json.dumps(msg))
                            except asyncio.TimeoutError:
                                pass
                            except Exception as e:
                                print(f"[auto_pilot] send error: {e}", flush=True)
                                return

                    async def recv_commands():
                        async for raw in ws:
                            try:
                                msg = json.loads(raw)
                            except Exception:
                                continue
                            mtype = msg.get("type", "")
                            if mtype == "start_autonomous":
                                wps      = [tuple(w) for w in msg.get("waypoints", [])]
                                boundary = [list(b) for b in msg.get("boundary",  [])]
                                cfg      = msg.get("config", {})
                                node.start_mission(wps, boundary, cfg)
                                await ws.send(json.dumps({
                                    "type":   "autonomous_status",
                                    "status": "STARTING",
                                    "log":    f"Mission received · {len(wps)} wps · {len(boundary)}-pt fence",
                                    "level":  "success",
                                }))
                            elif mtype == "stop_autonomous":
                                node.stop_mission()
                                await ws.send(json.dumps({
                                    "type":   "autonomous_status",
                                    "status": "ABORTED",
                                    "log":    "Mission aborted by operator",
                                    "level":  "warn",
                                }))

                    await asyncio.gather(send_status(), recv_commands())

            except Exception as e:
                print(f"[auto_pilot] Relay error: {e} — reconnecting in 3s", flush=True)
                await asyncio.sleep(3)

    await _inner()


# ── Entry point ───────────────────────────────────────────────────────────────
def main():
    rclpy.init()
    node = AutonomousNode()
    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()
    try:
        asyncio.run(ws_loop(node))
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_mission()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
