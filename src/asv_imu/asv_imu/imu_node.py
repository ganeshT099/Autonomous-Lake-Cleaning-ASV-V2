#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import Imu, MagneticField
from std_msgs.msg import String
import math
import threading

try:
    import board
    import busio
    from adafruit_bno08x import (
        BNO_REPORT_ROTATION_VECTOR,
        BNO_REPORT_GYROSCOPE,
        BNO_REPORT_ACCELEROMETER,
        BNO_REPORT_MAGNETOMETER,
    )
    from adafruit_bno08x.i2c import BNO08X_I2C
    BNO_AVAILABLE = True
except ImportError:
    BNO_AVAILABLE = False

class IMUNode(Node):
    def __init__(self):
        super().__init__('asv_imu_node')

        self.declare_parameter('frame_id',   'imu_link')
        self.declare_parameter('publish_hz',  50.0)
        self.declare_parameter('i2c_address', 0x4A)

        self.frame_id = self.get_parameter('frame_id').value
        self.pub_hz   = self.get_parameter('publish_hz').value

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=10)

        self.imu_pub    = self.create_publisher(Imu,          '/asv/imu/data',   sensor_qos)
        self.mag_pub    = self.create_publisher(MagneticField, '/asv/imu/mag',    sensor_qos)
        self.status_pub = self.create_publisher(String,        '/asv/imu/status', 10)

        if not BNO_AVAILABLE:
            self.get_logger().warn('Adafruit BNO08x library not found — hardware not connected')
            self.bno = None
        else:
            try:
                i2c = busio.I2C(board.SCL, board.SDA, frequency=400_000)
                self.bno = BNO08X_I2C(i2c)
                self.bno.enable_feature(BNO_REPORT_ROTATION_VECTOR)
                self.bno.enable_feature(BNO_REPORT_GYROSCOPE)
                self.bno.enable_feature(BNO_REPORT_ACCELEROMETER)
                self.bno.enable_feature(BNO_REPORT_MAGNETOMETER)
                self.get_logger().info('BNO085 initialised on I2C')
            except Exception as e:
                self.get_logger().error(f'BNO085 init failed: {e}')
                self.bno = None

        self.timer = self.create_timer(1.0 / self.pub_hz, self._publish)
        self.get_logger().info('IMU node started')

    def _publish(self):
        now = self.get_clock().now().to_msg()

        imu_msg = Imu()
        imu_msg.header.stamp    = now
        imu_msg.header.frame_id = self.frame_id

        mag_msg = MagneticField()
        mag_msg.header.stamp    = now
        mag_msg.header.frame_id = self.frame_id

        if self.bno is not None:
            try:
                qi, qj, qk, qr = self.bno.quaternion
                imu_msg.orientation.x = qi
                imu_msg.orientation.y = qj
                imu_msg.orientation.z = qk
                imu_msg.orientation.w = qr
                imu_msg.orientation_covariance = [0.0025,0,0, 0,0.0025,0, 0,0,0.0025]

                gx, gy, gz = self.bno.gyro
                imu_msg.angular_velocity.x = gx
                imu_msg.angular_velocity.y = gy
                imu_msg.angular_velocity.z = gz
                imu_msg.angular_velocity_covariance = [0.0001,0,0, 0,0.0001,0, 0,0,0.0001]

                ax, ay, az = self.bno.acceleration
                imu_msg.linear_acceleration.x = ax
                imu_msg.linear_acceleration.y = ay
                imu_msg.linear_acceleration.z = az
                imu_msg.linear_acceleration_covariance = [0.01,0,0, 0,0.01,0, 0,0,0.01]

                mx, my, mz = self.bno.magnetic
                mag_msg.magnetic_field.x = mx * 1e-6
                mag_msg.magnetic_field.y = my * 1e-6
                mag_msg.magnetic_field.z = mz * 1e-6
                mag_msg.magnetic_field_covariance = [1e-6,0,0, 0,1e-6,0, 0,0,1e-6]

                yaw = math.atan2(
                    2.0 * (qr * qk + qi * qj),
                    1.0 - 2.0 * (qj * qj + qk * qk))
                heading = math.degrees(yaw) % 360.0

                s = String()
                s.data = (f'[IMU] heading={heading:.1f}° | '
                          f'gyro=({gx:.3f},{gy:.3f},{gz:.3f}) | '
                          f'accel=({ax:.2f},{ay:.2f},{az:.2f})')
                self.status_pub.publish(s)

            except Exception as e:
                self.get_logger().warn(f'IMU read error: {e}')
        else:
            s = String()
            s.data = '[IMU] Waiting for hardware — connect BNO085 via I2C'
            self.status_pub.publish(s)

        self.imu_pub.publish(imu_msg)
        self.mag_pub.publish(mag_msg)

def main(args=None):
    rclpy.init(args=args)
    node = IMUNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
