import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import serial

from rclpy.qos import QoSProfile, ReliabilityPolicy


class IMUNode(Node):
    def __init__(self):
        super().__init__('imu_node')

        # 🔌 Serial setup
        self.ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)
        self.ser.reset_input_buffer()

        # ✅ QoS (VERY IMPORTANT)
        qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT
        )

        # 📡 Publisher
        self.publisher_ = self.create_publisher(
            Imu,
            '/asv/imu/data',
            qos
        )

        # ⏱ Timer (10 Hz)
        self.timer = self.create_timer(0.1, self.read_imu)

        # 🔧 Low-pass filter variables
        self.ax_f = 0.0
        self.ay_f = 0.0
        self.az_f = 0.0
        self.alpha = 0.8  # smoothing factor

        self.get_logger().info("✅ IMU node started")

    def read_imu(self):
        try:
            line = self.ser.readline()

            if not line:
                return

            try:
                line = line.decode('utf-8').strip()
            except:
                self.get_logger().warn("Decode error")
                return

            # ✅ Strict packet check (IMPORTANT)
            if line.count(',') != 5:
                self.get_logger().warn(f"⚠️ Bad packet: {line}")
                return

            data = line.split(',')

            try:
                ax, ay, az, gx, gy, gz = map(float, data)
            except Exception as e:
                self.get_logger().warn(f"Parse error: {e}")
                return

            # 🔧 Low-pass filter (smooth noise)
            self.ax_f = self.alpha * self.ax_f + (1 - self.alpha) * ax
            self.ay_f = self.alpha * self.ay_f + (1 - self.alpha) * ay
            self.az_f = self.alpha * self.az_f + (1 - self.alpha) * az

            # 🔧 Unit conversion
            ax_m = self.ax_f * 9.81
            ay_m = self.ay_f * 9.81
            az_m = self.az_f * 9.81

            gx_r = gx * 0.01745
            gy_r = gy * 0.01745
            gz_r = gz * 0.01745

            # 📋 Clean debug log (like GPS)
            self.get_logger().info(
                f"[IMU] ax={ax_m:.2f} ay={ay_m:.2f} az={az_m:.2f} | "
                f"gx={gx_r:.2f} gy={gy_r:.2f} gz={gz_r:.2f}"
            )

            # 📡 ROS message
            msg = Imu()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "imu_link"

            msg.linear_acceleration.x = ax_m
            msg.linear_acceleration.y = ay_m
            msg.linear_acceleration.z = az_m

            msg.angular_velocity.x = gx_r
            msg.angular_velocity.y = gy_r
            msg.angular_velocity.z = gz_r

            msg.orientation_covariance[0] = -1.0
            msg.angular_velocity_covariance[0] = -1.0
            msg.linear_acceleration_covariance[0] = -1.0

            self.publisher_.publish(msg)

        except Exception as e:
            self.get_logger().error(f"🔥 IMU ERROR: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = IMUNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
