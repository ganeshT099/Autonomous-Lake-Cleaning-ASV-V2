#!/usr/bin/env python3
"""
ASV GPS Node - Neo M9N
========================
Reads NMEA from Neo M9N via USB
Port: /dev/ttyACM0
Baud: 38400

Publishes:
  /asv/gps/fix    -> NavSatFix
  /asv/gps/vel    -> TwistStamped
  /asv/gps/status -> String

NMEA sentences parsed:
  GGA -> position, altitude, satellites, HDOP
  RMC -> speed, heading, date
"""

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
import signal
import sys


class GPSNode(Node):
    def __init__(self):
        super().__init__('asv_gps_node')

        self.declare_parameter('port',       '/dev/ttyACM0')
        self.declare_parameter('baud_rate',   38400)
        self.declare_parameter('frame_id',   'gps_link')
        self.declare_parameter('publish_hz',  10.0)

        self.port     = self.get_parameter('port').value
        self.baud     = self.get_parameter('baud_rate').value
        self.frame_id = self.get_parameter('frame_id').value
        self.pub_hz   = self.get_parameter('publish_hz').value

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=10)

        self.fix_pub    = self.create_publisher(
            NavSatFix,    '/asv/gps/fix',    sensor_qos)
        self.vel_pub    = self.create_publisher(
            TwistStamped, '/asv/gps/vel',    sensor_qos)
        self.status_pub = self.create_publisher(
            String,       '/asv/gps/status', 10)

        self.serial_conn = None
        self.running     = False
        self.lock        = threading.Lock()

        self.latitude    = 0.0
        self.longitude   = 0.0
        self.altitude    = 0.0
        self.speed_ms    = 0.0
        self.heading     = 0.0
        self.satellites  = 0
        self.hdop        = 99.99
        self.gps_qual    = 0
        self.has_fix     = False

        self._connect_serial()

        self.read_thread = threading.Thread(
            target=self._read_loop, daemon=True)
        self.read_thread.start()

        self.create_timer(1.0 / self.pub_hz, self._publish)

        self.get_logger().info(
            f'GPS node started | port={self.port} baud={self.baud}')
        self.get_logger().info('Waiting for GPS fix — go outside!')

    def _connect_serial(self):
        while True:
            try:
                self.serial_conn = serial.Serial(
                    self.port, self.baud, timeout=1.0)
                self.running = True
                self.get_logger().info(
                    f'GPS serial connected on {self.port} ✅')
                break
            except serial.SerialException as e:
                self.get_logger().error(
                    f'Cannot open {self.port}: {e} retrying in 3s...')
                time.sleep(3.0)

    def _read_loop(self):
        while self.running:
            try:
                raw = self.serial_conn.readline().decode(
                    'ascii', errors='replace').strip()

                if not raw.startswith('$'):
                    continue

                msg = pynmea2.parse(raw)
                now = self.get_clock().now().to_msg()

                if isinstance(msg, pynmea2.GGA):
                    with self.lock:
                        self.gps_qual = int(msg.gps_qual) \
                            if msg.gps_qual else 0
                        self.has_fix = self.gps_qual > 0

                        if self.has_fix and msg.latitude:
                            self.latitude  = float(msg.latitude)
                            self.longitude = float(msg.longitude)
                            self.altitude  = float(msg.altitude) \
                                if msg.altitude else 0.0

                        self.satellites = int(msg.num_sats) \
                            if msg.num_sats else 0
                        self.hdop = float(msg.horizontal_dil) \
                            if msg.horizontal_dil else 99.99

                elif isinstance(msg, pynmea2.RMC):
                    with self.lock:
                        if msg.spd_over_grnd is not None:
                            self.speed_ms = float(
                                msg.spd_over_grnd) * 0.514444
                        if msg.true_course is not None:
                            self.heading = float(msg.true_course)

            except pynmea2.ParseError:
                pass
            except serial.SerialException as e:
                self.get_logger().warn(f'Serial error: {e}')
                self.running = False
                self._connect_serial()
            except Exception as e:
                self.get_logger().debug(f'Skip: {e}')

    def _publish(self):
        with self.lock:
            latitude   = self.latitude
            longitude  = self.longitude
            altitude   = self.altitude
            speed_ms   = self.speed_ms
            heading    = self.heading
            satellites = self.satellites
            hdop       = self.hdop
            gps_qual   = self.gps_qual
            has_fix    = self.has_fix

        now = self.get_clock().now().to_msg()

        fix_msg = NavSatFix()
        fix_msg.header.stamp    = now
        fix_msg.header.frame_id = self.frame_id
        fix_msg.latitude        = float(latitude)
        fix_msg.longitude       = float(longitude)
        fix_msg.altitude        = float(altitude)

        status = NavSatStatus()
        if gps_qual == 0:
            status.status = NavSatStatus.STATUS_NO_FIX
        elif gps_qual == 2:
            status.status = NavSatStatus.STATUS_SBAS_FIX
        else:
            status.status = NavSatStatus.STATUS_FIX
        status.service    = NavSatStatus.SERVICE_GPS
        fix_msg.status    = status

        # ── FIXED covariance — all must be float! ─────────────
        cov = float(hdop * 5.0) * float(hdop * 5.0)
        fix_msg.position_covariance = [
            cov,         0.0,         0.0,
            0.0,         cov,         0.0,
            0.0,         0.0,         cov * 4.0]
        fix_msg.position_covariance_type = \
            NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN

        vel_msg = TwistStamped()
        vel_msg.header.stamp    = now
        vel_msg.header.frame_id = self.frame_id
        vel_msg.twist.linear.x  = float(speed_ms)
        vel_msg.twist.angular.z = float(heading)

        self.fix_pub.publish(fix_msg)
        self.vel_pub.publish(vel_msg)

        s = String()
        if not has_fix:
            s.data = (f'[GPS] NO FIX | '
                      f'sats={satellites} '
                      f'hdop={hdop:.1f}')
        else:
            s.data = (f'[GPS] FIX! | '
                      f'lat={latitude:.6f} '
                      f'lon={longitude:.6f} '
                      f'alt={altitude:.1f}m | '
                      f'spd={speed_ms:.2f}m/s '
                      f'hdg={heading:.1f} | '
                      f'sats={satellites} '
                      f'hdop={hdop:.1f}')
        self.status_pub.publish(s)
        self.get_logger().info(s.data)

    def destroy_node(self):
        self.running = False
        if self.serial_conn and self.serial_conn.is_open:
            self.serial_conn.close()
            self.get_logger().info('GPS serial closed!')
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = GPSNode()

    def shutdown(sig, frame):
        print('\nShutting down GPS node...')
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass
        sys.exit(0)

    signal.signal(signal.SIGINT,  shutdown)
    signal.signal(signal.SIGTERM, shutdown)

    try:
        rclpy.spin(node)
    except SystemExit:
        pass


if __name__ == '__main__':
    main()
