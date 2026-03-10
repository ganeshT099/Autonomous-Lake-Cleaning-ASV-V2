#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import String, Float32
import serial
import threading
import time
import struct

class RadarNode(Node):
    def __init__(self):
        super().__init__('asv_radar_node')

        self.declare_parameter('port',         '/dev/ttyUSB0')
        self.declare_parameter('baud_rate',     115200)
        self.declare_parameter('frame_id',      'radar_link')
        self.declare_parameter('publish_hz',    10.0)
        self.declare_parameter('max_range',     9.0)
        self.declare_parameter('min_range',     0.15)

        self.port      = self.get_parameter('port').value
        self.baud      = self.get_parameter('baud_rate').value
        self.frame_id  = self.get_parameter('frame_id').value
        self.pub_hz    = self.get_parameter('publish_hz').value
        self.max_range = self.get_parameter('max_range').value
        self.min_range = self.get_parameter('min_range').value

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=10)

        self.pc_pub       = self.create_publisher(PointCloud2, '/asv/radar/pointcloud', sensor_qos)
        self.dist_pub     = self.create_publisher(Float32,     '/asv/radar/distance',   sensor_qos)
        self.presence_pub = self.create_publisher(String,      '/asv/radar/presence',   10)
        self.status_pub   = self.create_publisher(String,      '/asv/radar/status',     10)

        # Latest data
        self.distance  = 0.0
        self.presence  = False
        self.lock      = threading.Lock()
        self.running   = False
        self.ser       = None

        self._connect_serial()
        self.read_thread = threading.Thread(target=self._read_loop, daemon=True)
        self.read_thread.start()
        self.timer = self.create_timer(1.0 / self.pub_hz, self._publish)
        self.get_logger().info(f'DFRobot C4001 Radar node started | port={self.port}')

    def _connect_serial(self):
        while True:
            try:
                self.ser = serial.Serial(self.port, self.baud, timeout=1.0)
                self.running = True
                self.get_logger().info(f'Radar serial connected on {self.port}')
                break
            except serial.SerialException as e:
                self.get_logger().error(f'Radar serial error: {e} retrying in 3s')
                time.sleep(3.0)

    def _parse_line(self, line):
        """
        DFRobot C4001 UART output format:
        $JYBSS,<presence>,<distance>,<speed>,<SNR>*<checksum>
        Example: $JYBSS,1,0.82,0,0*FF
                 $JYBSS,0,0.00,0,0*FF  (no presence)
        """
        try:
            if not line.startswith('$JYBSS'):
                return
            # Strip checksum
            data = line.split('*')[0]
            parts = data.split(',')
            if len(parts) < 3:
                return
            presence = int(parts[1]) == 1
            distance = float(parts[2])
            with self.lock:
                self.presence = presence
                self.distance = distance if presence else 0.0
        except Exception as e:
            self.get_logger().debug(f'Parse skip: {e}')

    def _read_loop(self):
        while self.running:
            try:
                line = self.ser.readline().decode('ascii', errors='replace').strip()
                if line:
                    self._parse_line(line)
            except serial.SerialException as e:
                self.get_logger().warn(f'Radar serial error: {e}')
                self.running = False
                self._connect_serial()
            except Exception as e:
                self.get_logger().debug(f'Radar skip: {e}')

    def _publish(self):
        with self.lock:
            dist     = self.distance
            presence = self.presence

        now = self.get_clock().now().to_msg()

        # Distance topic
        d = Float32()
        d.data = dist
        self.dist_pub.publish(d)

        # PointCloud2 — single point ahead
        fields = [
            PointField(name='x',         offset=0,  datatype=PointField.FLOAT32, count=1),
            PointField(name='y',         offset=4,  datatype=PointField.FLOAT32, count=1),
            PointField(name='z',         offset=8,  datatype=PointField.FLOAT32, count=1),
            PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
        ]
        pc2 = PointCloud2()
        pc2.header.stamp    = now
        pc2.header.frame_id = self.frame_id
        pc2.height       = 1
        pc2.width        = 1 if presence else 0
        pc2.fields       = fields
        pc2.is_dense     = True
        pc2.is_bigendian = False
        pc2.point_step   = 16
        pc2.row_step     = 16 * pc2.width
        if presence:
            pc2.data = struct.pack('ffff', dist, 0.0, 0.0, 1.0)
        else:
            pc2.data = b''
        self.pc_pub.publish(pc2)

        # Presence + status
        p = String()
        p.data = 'PRESENT' if presence else 'CLEAR'
        self.presence_pub.publish(p)

        s = String()
        if presence:
            s.data = f'[RADAR C4001] OBSTACLE DETECTED | distance={dist:.2f}m'
        else:
            s.data = '[RADAR C4001] Area clear — no obstacles'
        self.status_pub.publish(s)

    def destroy_node(self):
        self.running = False
        if self.ser and self.ser.is_open:
            self.ser.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = RadarNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
