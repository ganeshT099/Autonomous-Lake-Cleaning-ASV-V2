import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import serial

class IMUNode(Node):
    def __init__(self):
        super().__init__('imu_node')

        self.ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)

        self.publisher_ = self.create_publisher(Imu, '/imu/data', 10)

        self.timer = self.create_timer(0.05, self.read_imu)

    def read_imu(self):
        try:
            line = self.ser.readline().decode(errors='ignore').strip()
            data = line.split(',')

            if len(data) == 6:
                ax, ay, az = map(float, data[:3])
                gx, gy, gz = map(float, data[3:])

                msg = Imu()

                # 🔥 REQUIRED HEADER
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.header.frame_id = "imu_link"

                # Acceleration (m/s^2)
                msg.linear_acceleration.x = ax * 9.81
                msg.linear_acceleration.y = ay * 9.81
                msg.linear_acceleration.z = az * 9.81

                # Gyro (rad/s)
                msg.angular_velocity.x = gx * 0.01745
                msg.angular_velocity.y = gy * 0.01745
                msg.angular_velocity.z = gz * 0.01745

                # 🔥 VERY IMPORTANT (fix buffer error)
                msg.orientation_covariance[0] = -1.0
                msg.angular_velocity_covariance[0] = -1.0
                msg.linear_acceleration_covariance[0] = -1.0

                self.publisher_.publish(msg)

        except Exception as e:
            self.get_logger().warn(f"Error: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = IMUNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
