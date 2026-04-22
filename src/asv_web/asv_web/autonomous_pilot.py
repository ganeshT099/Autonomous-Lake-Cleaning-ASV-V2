"""
Autonomous Mission Pilot — run on Jetson alongside the existing ws_bridge.

Connects to wss://asv-server.onrender.com/ws/auto_pilot
Handles start_autonomous / stop_autonomous commands from the browser.
Uses ROS 2 GPS + IMU to follow waypoints with drift correction.
Publishes cmd_vel to /asv/cmd_vel so the existing thruster_node drives the ESCs.

Run:
  source /opt/ros/humble/setup.bash
  source ~/asv_ws/install/setup.bash
  python3 autonomous_pilot.py
"""

import asyncio
import json
import math
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import NavSatFix, Imu
from geometry_msgs.msg import Twist
import websockets

RELAY_URL = "wss://asv-server-1.onrender.com/ws/auto_pilot"

# ── Physics helpers ─────────────────────────────────────────────────────────
DEG2RAD = math.pi / 180
RAD2DEG = 180 / math.pi
EARTH_R  = 6_371_000.0

def haversine(lat1, lon1, lat2, lon2):
    dlat = (lat2 - lat1) * DEG2RAD
    dlon = (lon2 - lon1) * DEG2RAD
    a = math.sin(dlat / 2) ** 2 + \
        math.cos(lat1 * DEG2RAD) * math.cos(lat2 * DEG2RAD) * math.sin(dlon / 2) ** 2
    return EARTH_R * 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))

def bearing_to(lat1, lon1, lat2, lon2):
    dlon = (lon2 - lon1) * DEG2RAD
    y = math.sin(dlon) * math.cos(lat2 * DEG2RAD)
    x = (math.cos(lat1 * DEG2RAD) * math.sin(lat2 * DEG2RAD)
         - math.sin(lat1 * DEG2RAD) * math.cos(lat2 * DEG2RAD) * math.cos(dlon))
    return math.atan2(y, x)

def wrap_angle(a):
    return ((a + math.pi) % (2 * math.pi) + 2 * math.pi) % (2 * math.pi) - math.pi

def cross_track_error(lat, lon, lat1, lon1, lat2, lon2):
    d13 = haversine(lat1, lon1, lat, lon)
    b13 = bearing_to(lat1, lon1, lat, lon)
    b12 = bearing_to(lat1, lon1, lat2, lon2)
    return EARTH_R * math.asin(
        max(-1.0, min(1.0, math.sin(d13 / EARTH_R) * math.sin(b13 - b12)))
    )

def point_in_polygon(lat, lon, polygon):
    """Ray-casting point-in-polygon. polygon = [[lat, lon], ...]"""
    inside = False
    n = len(polygon)
    j = n - 1
    for i in range(n):
        xi, yi = polygon[i][0], polygon[i][1]
        xj, yj = polygon[j][0], polygon[j][1]
        if ((yi > lon) != (yj > lon)) and \
           (lat < (xj - xi) * (lon - yi) / (yj - yi + 1e-12) + xi):
            inside = not inside
        j = i
    return inside


# ── ROS 2 node ───────────────────────────────────────────────────────────────
class AutonomousNode(Node):
    def __init__(self):
        super().__init__("autonomous_pilot")

        sensor_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )

        self.sub_gps = self.create_subscription(
            NavSatFix, "/asv/gps/fix", self._on_gps, sensor_qos
        )
        self.sub_imu = self.create_subscription(
            Imu, "/asv/imu", self._on_imu, 10
        )
        self.pub_cmd = self.create_publisher(Twist, "/asv/cmd_vel", 10)

        # Sensor state
        self.lat     = 0.0
        self.lon     = 0.0
        self.heading = 0.0    # radians, integrated from IMU
        self.gps_ok  = False

        # GPS-derived heading (more reliable than IMU-only for long missions)
        self.prev_lat = 0.0
        self.prev_lon = 0.0
        self.gps_heading = None   # None until first valid GPS motion

        # Mission state
        self.waypoints        = []
        self.wp_idx           = 0
        self.running          = False
        self.config           = {}
        self.boundary         = []   # [[lat, lon], ...] geofence polygon
        self.return_start_idx = None # geofence disabled after this wp index
        self.entry_complete   = False  # True once boat has entered boundary
        self.cte_history      = []
        self.lock             = threading.Lock()

        # Telemetry callback (set by main loop)
        self.on_status = None

        # PID integral state
        self._integral = 0.0
        self._prev_err = 0.0

        # Control loop timer: 10 Hz
        self.create_timer(0.1, self._control_loop)

        self.get_logger().info("AutonomousNode ready")

    # ── Sensor callbacks ──────────────────────────────────────────────────────

    def _on_gps(self, msg: NavSatFix):
        if math.isnan(msg.latitude) or math.isnan(msg.longitude):
            return
        new_lat, new_lon = msg.latitude, msg.longitude

        # Derive heading from GPS motion when moving (more accurate than IMU-only)
        if self.gps_ok:
            d = haversine(self.prev_lat, self.prev_lon, new_lat, new_lon)
            if d > 0.25:   # only update when moved > 25 cm (GPS noise threshold)
                gps_hdg = bearing_to(self.prev_lat, self.prev_lon, new_lat, new_lon)
                if self.gps_heading is None:
                    self.gps_heading = gps_hdg
                    self.heading = gps_hdg
                else:
                    # Low-pass filter: GPS-derived heading replaces IMU drift
                    self.gps_heading = wrap_angle(
                        self.gps_heading + wrap_angle(gps_hdg - self.gps_heading) * 0.4
                    )
                    # Sync IMU integration baseline to GPS-derived heading
                    self.heading = self.gps_heading
                self.prev_lat = new_lat
                self.prev_lon = new_lon

        self.lat    = new_lat
        self.lon    = new_lon
        if not self.gps_ok:
            self.prev_lat = new_lat
            self.prev_lon = new_lon
        self.gps_ok = True

    def _on_imu(self, msg: Imu):
        # IMU gyro integration — used between GPS updates, synced to GPS heading
        dt = 0.1
        self.heading += msg.angular_velocity.z * dt

    # ── Mission control ───────────────────────────────────────────────────────

    def start_mission(self, waypoints, boundary, config):
        with self.lock:
            self.waypoints        = waypoints
            self.boundary         = boundary
            self.wp_idx           = 0
            self.running          = True
            self.config           = config
            self.cte_history      = []
            self.return_start_idx = config.get("return_start_idx", None)
            self.entry_complete   = False   # reset: boat starts outside boundary
            self._integral        = 0.0
            self._prev_err        = 0.0
        self.get_logger().info(f"Mission started: {len(waypoints)} waypoints")

    def stop_mission(self):
        with self.lock:
            self.running = False
        self._publish_stop()
        self.get_logger().info("Mission stopped")

    def _publish_stop(self):
        self.pub_cmd.publish(Twist())

    # ── LOS lookahead ─────────────────────────────────────────────────────────

    def _lookahead_point(self, wps, wp_idx, lookahead_m):
        """
        Walk `lookahead_m` ahead on the path starting from wp_idx.
        Returns (lat, lon) of the lookahead point.
        """
        remaining = lookahead_m
        cur_lat, cur_lon = self.lat, self.lon

        for i in range(wp_idx, len(wps) - 1):
            seg = haversine(cur_lat, cur_lon, wps[i + 1][0], wps[i + 1][1])
            if seg >= remaining:
                frac = remaining / max(seg, 1e-9)
                return (
                    cur_lat + frac * (wps[i + 1][0] - cur_lat),
                    cur_lon + frac * (wps[i + 1][1] - cur_lon),
                )
            remaining -= seg
            cur_lat, cur_lon = wps[i + 1][0], wps[i + 1][1]

        return wps[min(wp_idx, len(wps) - 1)]

    # ── Control loop (10 Hz) ──────────────────────────────────────────────────

    def _control_loop(self):
        with self.lock:
            if not self.running or not self.gps_ok or not self.waypoints:
                return
            wp_idx           = self.wp_idx
            waypoints        = self.waypoints
            config           = self.config
            boundary         = self.boundary
            return_start_idx = self.return_start_idx

        max_speed = float(config.get("speed", 0.5))
        lookahead  = float(config.get("lookahead", 8.0))
        WP_THRESH  = max(1.5, max_speed * 2.0)   # acceptance radius scales with speed
        KP, KI, KD = 1.4, 0.008, 0.25
        dt = 0.1

        if wp_idx >= len(waypoints):
            self._finish_mission()
            return

        # ── Geofence check ────────────────────────────────────────────────────
        in_return_leg = (return_start_idx is not None and wp_idx >= return_start_idx)

        if boundary:
            currently_inside = point_in_polygon(self.lat, self.lon, boundary)
            # Mark entry once the boat crosses into the boundary for the first time
            if not self.entry_complete and currently_inside:
                with self.lock:
                    self.entry_complete = True
                self.get_logger().info("Geofence: boat entered boundary — enforcement active")
            # Only enforce geofence AFTER entry and NOT on the return leg
            if self.entry_complete and not in_return_leg and not currently_inside:
                self._publish_stop()
                with self.lock:
                    self.running = False
                if self.on_status:
                    self.on_status({
                        "type":   "autonomous_status",
                        "status": "GEOFENCE",
                        "log":    "GEOFENCE BREACH — mission aborted",
                        "level":  "error",
                        "pct":    round(wp_idx / len(waypoints) * 100, 1),
                    })
                return

        # ── Advance waypoint if within acceptance radius ───────────────────────
        while wp_idx < len(waypoints):
            d = haversine(self.lat, self.lon, waypoints[wp_idx][0], waypoints[wp_idx][1])
            if d < WP_THRESH:
                wp_idx += 1
            else:
                break

        with self.lock:
            self.wp_idx = wp_idx

        if wp_idx >= len(waypoints):
            self._finish_mission()
            return

        # ── LOS lookahead guidance ────────────────────────────────────────────
        lk_lat, lk_lon = self._lookahead_point(waypoints, wp_idx, lookahead)
        desired_bearing = bearing_to(self.lat, self.lon, lk_lat, lk_lon)
        heading_error   = wrap_angle(desired_bearing - self.heading)

        # PID on heading error
        self._integral = max(-1.5, min(1.5, self._integral + heading_error * dt))
        d_err = (heading_error - self._prev_err) / dt
        self._prev_err = heading_error
        raw_angular = KP * heading_error + KI * self._integral + KD * d_err
        angular_cmd = max(-1.0, min(1.0, raw_angular / math.pi))

        # Slow down proportionally in sharp turns (like a plane banking)
        linear_cmd = max_speed * max(0.2, 1.0 - 0.6 * abs(angular_cmd))

        cmd = Twist()
        cmd.linear.x  = linear_cmd
        cmd.angular.z = angular_cmd
        self.pub_cmd.publish(cmd)

        # ── CTE ────────────────────────────────────────────────────────────────
        cte = 0.0
        if wp_idx > 0:
            prev = waypoints[wp_idx - 1]
            curr = waypoints[wp_idx]
            cte = cross_track_error(self.lat, self.lon, prev[0], prev[1], curr[0], curr[1])
        self.cte_history.append(abs(cte))
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
            "cte":           round(cte, 3),
            "cte_avg":       round(cte_avg, 3),
            "cte_max":       round(cte_max, 3),
            "heading_error": round(math.degrees(heading_error), 2),
            "angular_cmd":   round(angular_cmd, 3),
            "linear_cmd":    round(linear_cmd, 3),
            "heading":       round(math.degrees(self.heading) % 360, 1),
            "dist_to_next":  round(dist, 1),
            "pct":           round(pct, 1),
            "lat":           self.lat,
            "lon":           self.lon,
        }

        if abs(cte) > 0.8:
            status["log"]   = f"CTE {'+' if cte >= 0 else ''}{cte:.2f}m  ω={angular_cmd:+.2f}"
            status["level"] = "warn"
        elif wp_idx % 10 == 0 and dist < WP_THRESH + 2:
            status["log"]   = f"WP {wp_idx}/{len(waypoints)}  {dist:.0f}m  hdg {math.degrees(self.heading)%360:.1f}°"
            status["level"] = "info"

        if self.on_status:
            self.on_status(status)

    def _finish_mission(self):
        with self.lock:
            self.running = False
        self._publish_stop()
        avg_cte = sum(self.cte_history) / len(self.cte_history) if self.cte_history else 0
        max_cte = max(self.cte_history) if self.cte_history else 0
        if self.on_status:
            self.on_status({
                "type":    "autonomous_status",
                "status":  "COMPLETE",
                "pct":     100.0,
                "cte_avg": round(avg_cte, 3),
                "cte_max": round(max_cte, 3),
                "log":     "MISSION COMPLETE ✓  All waypoints reached",
                "level":   "success",
            })
        self.get_logger().info("Mission complete")


# ── WebSocket client loop ────────────────────────────────────────────────────
async def ws_loop(node: AutonomousNode):
    status_queue: asyncio.Queue = asyncio.Queue()

    def on_status(msg):
        asyncio.get_event_loop().call_soon_threadsafe(status_queue.put_nowait, msg)

    node.on_status = on_status

    while True:
        try:
            async with websockets.connect(RELAY_URL, ping_interval=20) as ws:
                print(f"[auto_pilot] Connected to relay")

                async def send_status():
                    while True:
                        try:
                            msg = await asyncio.wait_for(status_queue.get(), timeout=0.5)
                            await ws.send(json.dumps(msg))
                        except asyncio.TimeoutError:
                            pass
                        except Exception as e:
                            print(f"[auto_pilot] Send error: {e}")
                            break

                async def recv_commands():
                    async for raw in ws:
                        try:
                            msg = json.loads(raw)
                        except Exception:
                            continue
                        mtype = msg.get("type", "")
                        if mtype == "start_autonomous":
                            wps      = msg.get("waypoints", [])
                            boundary = msg.get("boundary", [])
                            cfg      = msg.get("config", {})
                            node.start_mission(wps, boundary, cfg)
                            await ws.send(json.dumps({
                                "type":   "autonomous_status",
                                "status": "STARTING",
                                "log":    f"Mission received · {len(wps)} waypoints",
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
            print(f"[auto_pilot] Relay error: {e}  — reconnecting in 3s")
            await asyncio.sleep(3)


# ── Entry point ──────────────────────────────────────────────────────────────
def main():
    rclpy.init()
    node = AutonomousNode()

    ros_thread = threading.Thread(
        target=lambda: rclpy.spin(node), daemon=True
    )
    ros_thread.start()

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
