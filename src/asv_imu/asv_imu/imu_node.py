import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import serial
from rclpy.qos import QoSProfile

class IMUNode(Node):
    def __init__(self):
        super().__init__('imu_node')

        # Serial
        self.ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)

        # ✅ MATCH SYSTEM QoS (RELIABLE)
        qos = QoSProfile(depth=10)

        self.publisher_ = self.create_publisher(Imu, '/asv/imu/data', qos)

        # ✅ Stable rate
        self.timer = self.create_timer(0.2, self.read_imu)  # 5 Hz

    def read_imu(self):
        try:
            line = self.ser.readline().decode(errors='ignore').strip()

            if not line or ',' not in line:
                return

            data = line.split(',')

            if len(data) != 6:
                return

            ax, ay, az = map(float, data[:3])
            gx, gy, gz = map(float, data[3:])

            msg = Imu()

            # Header
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "imu_link"

            # Acceleration
            msg.linear_acceleration.x = ax * 9.81
            msg.linear_acceleration.y = ay * 9.81
            msg.linear_acceleration.z = az * 9.81

            # Gyro
            msg.angular_velocity.x = gx * 0.01745
            msg.angular_velocity.y = gy * 0.01745
            msg.angular_velocity.z = gz * 0.01745

            # Required fields
            msg.orientation_covariance[0] = -1.0
            msg.angular_velocity_covariance[0] = -1.0
            msg.linear_acceleration_covariance[0] = -1.0

            self.publisher_.publish(msg)

        except:
            pass


def main(args=None):
    rclpy.init(args=args)
    node = IMUNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
