#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import NavSatFix, NavSatStatus
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import String

import serial
import pynmea2
import threading
import time
import math
import signal
import sys


class GPSNode(Node):
    def __init__(self):
        super().__init__('asv_gps_node')

        # PARAMETERS
        self.declare_parameter('port', '/dev/ttyACM0')
        self.declare_parameter('baud_rate', 38400)
        self.declare_parameter('frame_id', 'gps_link')
        self.declare_parameter('publish_hz', 10.0)

        self.port = self.get_parameter('port').value
        self.baud = self.get_parameter('baud_rate').value
        self.frame_id = self.get_parameter('frame_id').value
        self.pub_hz = self.get_parameter('publish_hz').value

        # QoS (Sensor optimized)
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=10)

        # Publishers
        self.fix_pub = self.create_publisher(
            NavSatFix, '/asv/gps/fix', sensor_qos)
        self.vel_pub = self.create_publisher(
            TwistStamped, '/asv/gps/vel', sensor_qos)
        self.status_pub = self.create_publisher(
            String, '/asv/gps/status', 10)

        # DATA STORAGE
        self.serial_conn = None
        self.running = False
        self.lock = threading.Lock()

        self.latitude = 0.0
        self.longitude = 0.0
        self.altitude = 0.0
        self.speed_ms = 0.0
        self.heading = 0.0
        self.satellites = 0
        self.hdop = 99.99
        self.gps_qual = 0
        self.has_fix = False

        # CONNECT SERIAL
        self._connect_serial()

        # THREAD
        self.read_thread = threading.Thread(
            target=self._read_loop, daemon=True)
        self.read_thread.start()

        # TIMER
        self.create_timer(1.0 / self.pub_hz, self._publish)

        self.get_logger().info(
            f'🚀 GPS node started | {self.port} @ {self.baud}')
        self.get_logger().info('📡 Waiting for GPS fix...')

    # ---------------- SERIAL CONNECT ----------------
    def _connect_serial(self):
        while True:
            try:
                self.serial_conn = serial.Serial(
                    self.port, self.baud, timeout=1.0)
                self.running = True
                self.get_logger().info(
                    f'✅ Connected to GPS on {self.port}')
                break
            except serial.SerialException as e:
                self.get_logger().error(
                    f'❌ Serial error: {e} | retrying...')
                time.sleep(3)

    # ---------------- READ LOOP ----------------
    def _read_loop(self):
        while self.running:
            try:
                line = self.serial_conn.readline().decode(
                    'ascii', errors='ignore').strip()

                if not line.startswith('$'):
                    continue

                msg = pynmea2.parse(line)

                # -------- GGA (POSITION) --------
                if isinstance(msg, pynmea2.GGA):
                    with self.lock:
                        self.gps_qual = int(msg.gps_qual) if msg.gps_qual else 0
                        self.has_fix = self.gps_qual > 0

                        if self.has_fix and msg.latitude:
                            self.latitude = msg.latitude
                            self.longitude = msg.longitude
                            self.altitude = float(msg.altitude) if msg.altitude else 0.0

                        self.satellites = int(msg.num_sats) if msg.num_sats else 0
                        self.hdop = float(msg.horizontal_dil) if msg.horizontal_dil else 99.99

                # -------- RMC (SPEED + HEADING) --------
                elif isinstance(msg, pynmea2.RMC):
                    with self.lock:
                        if msg.spd_over_grnd:
                            self.speed_ms = float(msg.spd_over_grnd) * 0.514444
                        if msg.true_course:
                            self.heading = float(msg.true_course)

            except pynmea2.ParseError:
                pass
            except serial.SerialException as e:
                self.get_logger().warn(f'Serial lost: {e}')
                self.running = False
                self._connect_serial()
            except Exception:
                pass

            time.sleep(0.001)  # CPU protection

    # ---------------- PUBLISH ----------------
    def _publish(self):
        with self.lock:
            lat = self.latitude
            lon = self.longitude
            alt = self.altitude
            speed = self.speed_ms
            heading = self.heading
            sats = self.satellites
            hdop = self.hdop
            qual = self.gps_qual
            fix = self.has_fix

        now = self.get_clock().now().to_msg()

        # ---------- NavSatFix ----------
        fix_msg = NavSatFix()
        fix_msg.header.stamp = now
        fix_msg.header.frame_id = self.frame_id
        fix_msg.latitude = float(lat)
        fix_msg.longitude = float(lon)
        fix_msg.altitude = float(alt)

        status = NavSatStatus()
        if qual == 0:
            status.status = NavSatStatus.STATUS_NO_FIX
        elif qual == 2:
            status.status = NavSatStatus.STATUS_SBAS_FIX
        else:
            status.status = NavSatStatus.STATUS_FIX
        status.service = NavSatStatus.SERVICE_GPS
        fix_msg.status = status

        # Covariance
        cov = float(hdop * 5.0) ** 2
        fix_msg.position_covariance = [
            cov, 0.0, 0.0,
            0.0, cov, 0.0,
            0.0, 0.0, cov * 4.0
        ]
        fix_msg.position_covariance_type = \
            NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN

        # ---------- Velocity ----------
        vel_msg = TwistStamped()
        vel_msg.header.stamp = now
        vel_msg.header.frame_id = self.frame_id
        vel_msg.twist.linear.x = float(speed)
        vel_msg.twist.angular.z = math.radians(heading)  # FIXED

        # ---------- Publish ----------
        self.fix_pub.publish(fix_msg)
        self.vel_pub.publish(vel_msg)

        # ---------- Status ----------
        status_msg = String()

        if not fix:
            status_msg.data = f"[GPS] NO FIX | sats={sats} hdop={hdop:.1f}"
        else:
            status_msg.data = (
                f"[GPS] FIX | lat={lat:.6f} lon={lon:.6f} "
                f"spd={speed:.2f}m/s hdg={heading:.1f} "
                f"sats={sats}"
            )

        self.status_pub.publish(status_msg)

        self.get_logger().info(
            status_msg.data,
            throttle_duration_sec=2.0
        )

    # ---------------- CLEANUP ----------------
    def destroy_node(self):
        self.running = False
        if self.serial_conn and self.serial_conn.is_open:
            self.serial_conn.close()
        super().destroy_node()


# ---------------- MAIN ----------------
def main(args=None):
    rclpy.init(args=args)
    node = GPSNode()

    def shutdown(sig, frame):
        print("\nShutting down GPS node...")
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(0)

    signal.signal(signal.SIGINT, shutdown)
    signal.signal(signal.SIGTERM, shutdown)

    rclpy.spin(node)


if __name__ == '__main__':
    main()
