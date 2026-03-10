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

class GPSNode(Node):
    def __init__(self):
        super().__init__('asv_gps_node')
        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baud_rate', 38400)
        self.declare_parameter('frame_id', 'gps_link')
        self.declare_parameter('publish_hz', 10.0)
        self.port     = self.get_parameter('port').value
        self.baud     = self.get_parameter('baud_rate').value
        self.frame_id = self.get_parameter('frame_id').value
        self.pub_hz   = self.get_parameter('publish_hz').value

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE, depth=10)

        self.fix_pub    = self.create_publisher(NavSatFix,    '/asv/gps/fix',    sensor_qos)
        self.vel_pub    = self.create_publisher(TwistStamped, '/asv/gps/vel',    sensor_qos)
        self.status_pub = self.create_publisher(String,       '/asv/gps/status', 10)

        self.serial_conn = None
        self.running     = False
        self.fix_msg     = NavSatFix()
        self.vel_msg     = TwistStamped()
        self.lock        = threading.Lock()

        self._connect_serial()
        self.read_thread = threading.Thread(target=self._read_loop, daemon=True)
        self.read_thread.start()
        self.timer = self.create_timer(1.0 / self.pub_hz, self._publish)
        self.get_logger().info(f'GPS node started | port={self.port} baud={self.baud}')

    def _connect_serial(self):
        while True:
            try:
                self.serial_conn = serial.Serial(self.port, self.baud, timeout=1.0)
                self.running = True
                self.get_logger().info(f'Serial connected on {self.port}')
                break
            except serial.SerialException as e:
                self.get_logger().error(f'Cannot open {self.port}: {e} retrying in 3s')
                time.sleep(3.0)

    def _read_loop(self):
        while self.running:
            try:
                raw = self.serial_conn.readline().decode('ascii', errors='replace').strip()
                if not raw.startswith('$'):
                    continue
                msg = pynmea2.parse(raw)
                now = self.get_clock().now().to_msg()
                if isinstance(msg, pynmea2.GGA):
                    with self.lock:
                        self.fix_msg.header.stamp    = now
                        self.fix_msg.header.frame_id = self.frame_id
                        if msg.latitude and msg.longitude:
                            self.fix_msg.latitude  = float(msg.latitude)
                            self.fix_msg.longitude = float(msg.longitude)
                            self.fix_msg.altitude  = float(msg.altitude) if msg.altitude else 0.0
                        gps_qual = int(msg.gps_qual) if msg.gps_qual else 0
                        status = NavSatStatus()
                        status.status = NavSatStatus.STATUS_NO_FIX if gps_qual == 0 else NavSatStatus.STATUS_FIX
                        status.service = NavSatStatus.SERVICE_GPS
                        self.fix_msg.status = status
                        hdop = float(msg.horizontal_dil) if msg.horizontal_dil else 1.0
                        cov  = (hdop * 5.0) ** 2
                        self.fix_msg.position_covariance = [cov,0,0, 0,cov,0, 0,0,cov*4]
                        self.fix_msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN
                elif isinstance(msg, pynmea2.RMC):
                    with self.lock:
                        self.vel_msg.header.stamp    = now
                        self.vel_msg.header.frame_id = self.frame_id
                        if msg.spd_over_grnd is not None:
                            self.vel_msg.twist.linear.x = float(msg.spd_over_grnd) * 0.514444
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
            fix = self.fix_msg
            vel = self.vel_msg
        self.fix_pub.publish(fix)
        self.vel_pub.publish(vel)
        status_labels = {-1:'NO_FIX', 0:'NO_FIX', 1:'GPS_FIX', 2:'SBAS_FIX', 18:'DGPS_FIX'}
        label = status_labels.get(fix.status.status, 'GPS_FIX')
        s = String()
        s.data = f'[GPS] {label} | lat={fix.latitude:.6f} lon={fix.longitude:.6f} alt={fix.altitude:.1f}m'
        self.status_pub.publish(s)

    def destroy_node(self):
        self.running = False
        if self.serial_conn and self.serial_conn.is_open:
            self.serial_conn.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = GPSNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
