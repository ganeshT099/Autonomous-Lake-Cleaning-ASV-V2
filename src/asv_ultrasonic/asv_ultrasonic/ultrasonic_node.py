#!/usr/bin/env python3
"""
ASV Ultrasonic + Radar ESP32 Bridge Node
Reads combined data from ESP32 over USB Serial
Format: $ASV,F:0.82,R:1.20,L:1.45,RDR:1.20,SAFE*FF
Publishes:
  /asv/ultrasonic/front   (sensor_msgs/Range)
  /asv/ultrasonic/right   (sensor_msgs/Range)
  /asv/ultrasonic/left    (sensor_msgs/Range)
  /asv/ultrasonic/safety  (std_msgs/String) SAFE/WARNING/DANGER
  /asv/ultrasonic/status  (std_msgs/String)
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import Range
from std_msgs.msg import String
import serial
import threading
import time
import math

class ESP32BridgeNode(Node):
    def __init__(self):
        super().__init__('asv_ultrasonic_node')

        self.declare_parameter('port',              '/dev/ttyUSB0')
        self.declare_parameter('baud_rate',          115200)
        self.declare_parameter('publish_hz',         10.0)
        self.declare_parameter('danger_threshold',   0.5)
        self.declare_parameter('warning_threshold',  1.0)

        self.port      = self.get_parameter('port').value
        self.baud      = self.get_parameter('baud_rate').value
        self.pub_hz    = self.get_parameter('publish_hz').value
        self.danger    = self.get_parameter('danger_threshold').value
        self.warning   = self.get_parameter('warning_threshold').value

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=10)

        self.front_pub  = self.create_publisher(Range,  '/asv/ultrasonic/front',  sensor_qos)
        self.right_pub  = self.create_publisher(Range,  '/asv/ultrasonic/right',  sensor_qos)
        self.left_pub   = self.create_publisher(Range,  '/asv/ultrasonic/left',   sensor_qos)
        self.safety_pub = self.create_publisher(String, '/asv/ultrasonic/safety', 10)
        self.status_pub = self.create_publisher(String, '/asv/ultrasonic/status', 10)

        # Latest data
        self.data = {'F': -1.0, 'R': -1.0, 'L': -1.0, 'RDR': -1.0, 'SAFETY': 'SAFE'}
        self.lock = threading.Lock()

        self._connect_serial()
        self.read_thread = threading.Thread(target=self._read_loop, daemon=True)
        self.read_thread.start()
        self.timer = self.create_timer(1.0 / self.pub_hz, self._publish)
        self.get_logger().info(f'ESP32 Bridge node started | port={self.port}')

    def _connect_serial(self):
        while True:
            try:
                self.ser = serial.Serial(self.port, self.baud, timeout=1.0)
                self.get_logger().info(f'ESP32 connected on {self.port}')
                break
            except serial.SerialException as e:
                self.get_logger().error(f'ESP32 serial error: {e} retrying in 3s')
                time.sleep(3.0)

    def _parse(self, line):
        """Parse: $ASV,F:0.82,R:1.20,L:1.45,RDR:1.20,SAFE*FF"""
        try:
            if not line.startswith('$ASV'):
                return
            # Strip checksum
            line = line.split('*')[0].strip()
            parts = line.split(',')
            data = {}
            for part in parts[1:]:
                if ':' in part:
                    key, val = part.split(':')
                    data[key] = float(val)
                else:
                    data['SAFETY'] = part
            with self.lock:
                self.data.update(data)
        except Exception as e:
            self.get_logger().debug(f'Parse skip: {e}')

    def _read_loop(self):
        while True:
            try:
                line = self.ser.readline().decode('ascii', errors='replace').strip()
                if line:
                    self._parse(line)
            except serial.SerialException as e:
                self.get_logger().warn(f'ESP32 serial error: {e}')
                self._connect_serial()
            except Exception as e:
                self.get_logger().debug(f'Skip: {e}')

    def _make_range(self, distance, frame_id):
        msg = Range()
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = frame_id
        msg.radiation_type  = Range.ULTRASOUND
        msg.field_of_view   = math.radians(15.0)
        msg.min_range       = 0.25
        msg.max_range       = 4.5
        msg.range           = float(distance) if distance > 0 else float('inf')
        return msg

    def _publish(self):
        with self.lock:
            d = dict(self.data)

        self.front_pub.publish(self._make_range(d.get('F',  -1), 'ultrasonic_front_link'))
        self.right_pub.publish(self._make_range(d.get('R',  -1), 'ultrasonic_right_link'))
        self.left_pub.publish( self._make_range(d.get('L',  -1), 'ultrasonic_left_link'))

        safety = String()
        safety.data = d.get('SAFETY', 'SAFE')
        self.safety_pub.publish(safety)

        def fmt(v): return f'{v:.2f}m' if v > 0 else 'NO_ECHO'
        status = String()
        level = d.get('SAFETY', 'SAFE')
        status.data = (
            f'[ESP32] {level} | '
            f'F={fmt(d.get("F",-1))} '
            f'R={fmt(d.get("R",-1))} '
            f'L={fmt(d.get("L",-1))} '
            f'RDR={fmt(d.get("RDR",-1))}'
        )
        self.status_pub.publish(status)

def main(args=None):
    rclpy.init(args=args)
    node = ESP32BridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
