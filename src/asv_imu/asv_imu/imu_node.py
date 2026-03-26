#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import smbus
import time
import struct
import math

I2C_BUS = 7
BNO_ADDR = 0x4B

class BNO085:
    def __init__(self):
        self.bus = smbus.SMBus(I2C_BUS)
        self.seq = [0]*6

        self.reset()
        time.sleep(1)

        self.enable_rotation_vector()
        time.sleep(0.5)

    def reset(self):
        try:
            self.bus.write_i2c_block_data(BNO_ADDR, 0, [0x05, 0, 0, 0, 0])
        except:
            pass

    def send_packet(self, channel, data):
        length = len(data) + 4
        packet = [
            length & 0xFF,
            (length >> 8) & 0xFF,
            channel,
            self.seq[channel]
        ] + data

        self.seq[channel] = (self.seq[channel] + 1) & 0xFF
        self.bus.write_i2c_block_data(BNO_ADDR, 0, packet)

    def enable_rotation_vector(self):
        interval = 10000  # 100Hz

        data = [
            0xFD,
            0x05,
            0,0,
            0,0,
            interval & 0xFF,
            (interval >> 8) & 0xFF,
            (interval >> 16) & 0xFF,
            (interval >> 24) & 0xFF,
            0,0,0,0,
            0,0,0,0
        ]

        self.send_packet(2, data)

    def read_quaternion(self):
        try:
            header = self.bus.read_i2c_block_data(BNO_ADDR, 0, 4)
            length = (header[1] << 8 | header[0]) & 0x7FFF

            if length <= 4 or length > 50:
                return None

            data = self.bus.read_i2c_block_data(BNO_ADDR, 0, length)

            report_id = data[4]

            if report_id == 0x05:
                i = struct.unpack_from('<h', bytes(data), 6)[0] / 16384.0
                j = struct.unpack_from('<h', bytes(data), 8)[0] / 16384.0
                k = struct.unpack_from('<h', bytes(data),10)[0] / 16384.0
                r = struct.unpack_from('<h', bytes(data),12)[0] / 16384.0

                return (i, j, k, r)

        except Exception:
            return None

        return None


class IMUNode(Node):
    def __init__(self):
        super().__init__('asv_imu_node')

        try:
            self.bno = BNO085()
            self.get_logger().info("IMU connected on I2C ✅")
        except Exception as e:
            self.get_logger().error(f"IMU init failed: {e}")
            self.bno = None

        self.timer = self.create_timer(0.1, self.loop)

    def loop(self):
        if self.bno is None:
            return

        for _ in range(5):  # retry multiple times
            quat = self.bno.read_quaternion()

            if quat:
                qi, qj, qk, qr = quat

                yaw = math.atan2(
                    2.0 * (qr*qk + qi*qj),
                    1.0 - 2.0 * (qj*qj + qk*qk)
                )

                heading = math.degrees(yaw) % 360.0

                self.get_logger().info(f"[IMU] Heading: {heading:.2f}")
                return

        # If no data
        self.get_logger().info("[IMU] No data yet...")


def main():
    rclpy.init()
    node = IMUNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
