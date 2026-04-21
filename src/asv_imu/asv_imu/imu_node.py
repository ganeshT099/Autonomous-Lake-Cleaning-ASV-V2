import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import serial
import math

from rclpy.qos import QoSProfile, ReliabilityPolicy


class IMUNode(Node):
    def __init__(self):
        super().__init__('imu_node')

        # 🔌 Serial
        self.ser = serial.Serial('/dev/imu', 115200, timeout=1)
        self.ser.reset_input_buffer()

        # QoS
        qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT
        )

        # Publisher
        self.publisher_ = self.create_publisher(Imu, '/asv/imu/data', qos)

        # Timer
        self.timer = self.create_timer(0.1, self.read_imu)

        # 🔧 Filter
        self.ax_f = 0.0
        self.ay_f = 0.0
        self.az_f = 0.0
        self.alpha = 0.8

        # 🔥 Yaw
        self.yaw = 0.0
        self.last_time = self.get_clock().now()

        # 🔥 CALIBRATION
        self.gz_bias = 0.0
        self.calibrated = False
        self.samples = []

        self.get_logger().info("🚀 IMU node started (CALIBRATING...)")

    def read_imu(self):
        try:
            line = self.ser.readline()
            if not line:
                return

            try:
                line = line.decode('utf-8').strip()
            except:
                return

            if line.count(',') != 5:
                return

            data = line.split(',')

            try:
                ax, ay, az, gx, gy, gz = map(float, data)
            except:
                return

            # 🔧 Filter accel
            self.ax_f = self.alpha * self.ax_f + (1 - self.alpha) * ax
            self.ay_f = self.alpha * self.ay_f + (1 - self.alpha) * ay
            self.az_f = self.alpha * self.az_f + (1 - self.alpha) * az

            ax_m = self.ax_f * 9.81
            ay_m = self.ay_f * 9.81
            az_m = self.az_f * 9.81

            # 🔧 Gyro rad/s
            gx_r = gx * 0.01745
            gy_r = gy * 0.01745
            gz_r = gz * 0.01745

            # ---------------- CALIBRATION ----------------
            if not self.calibrated:
                self.samples.append(gz_r)

                if len(self.samples) >= 100:
                    self.gz_bias = sum(self.samples) / len(self.samples)
                    self.calibrated = True

                    self.yaw = 0.0  # ✅ FIX 4 (reset yaw)

                    self.get_logger().info(
                        f"✅ Calibration done! Gyro bias = {self.gz_bias:.5f}"
                    )

                return

            # ---------------- TIME ----------------
            current_time = self.get_clock().now()
            dt = (current_time - self.last_time).nanoseconds * 1e-9
            self.last_time = current_time

            # ✅ FIX 3 (dt safety)
            if dt <= 0 or dt > 0.2:
                dt = 0.1

            # ---------------- YAW ----------------
            gz_corrected = gz_r - self.gz_bias

            # deadband
            if abs(gz_corrected) < 0.03:
                gz_corrected = 0.0

            self.yaw += gz_corrected * dt

            # ✅ FIX 2 (drift damping)
            if abs(gz_corrected) < 0.005:
                self.yaw *= 0.999

            # normalize
            while self.yaw > math.pi:
                self.yaw -= 2 * math.pi
            while self.yaw < -math.pi:
                self.yaw += 2 * math.pi

            # ---------------- QUATERNION ----------------
            cy = math.cos(self.yaw * 0.5)
            sy = math.sin(self.yaw * 0.5)

            # ---------------- MESSAGE ----------------
            msg = Imu()
            msg.header.stamp = current_time.to_msg()
            msg.header.frame_id = "imu_link"

            msg.linear_acceleration.x = ax_m
            msg.linear_acceleration.y = ay_m
            msg.linear_acceleration.z = az_m

            msg.angular_velocity.x = gx_r
            msg.angular_velocity.y = gy_r

            # ✅ FIX 1 (use corrected gyro)
            msg.angular_velocity.z = gz_corrected

            msg.orientation.x = 0.0
            msg.orientation.y = 0.0
            msg.orientation.z = sy
            msg.orientation.w = cy

            msg.orientation_covariance[0] = 0.01
            msg.angular_velocity_covariance[0] = 0.01
            msg.linear_acceleration_covariance[0] = 0.01

            self.publisher_.publish(msg)

            # 🔥 Clean log
            self.get_logger().info(
                f"[IMU] yaw={math.degrees(self.yaw):.1f} deg",
                throttle_duration_sec=1.0
            )

        except Exception as e:
            self.get_logger().error(f"🔥 IMU ERROR: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = IMUNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
