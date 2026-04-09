import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from evdev import InputDevice, ecodes
import threading


class EvdevJoystick(Node):

    def __init__(self):
        super().__init__('evdev_joystick')

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.device = InputDevice('/dev/input/event0')
        self.get_logger().info(f"Using device: {self.device.path}")

        # STATE
        self.forward = 0
        self.reverse = 0
        self.steering = 0.0
        self.deadman = 0

        # THREAD (non-blocking)
        threading.Thread(target=self.read_loop, daemon=True).start()

        # PUBLISH LOOP
        self.create_timer(0.05, self.publish_cmd)

    def read_loop(self):
        for event in self.device.read_loop():

            if event.type == ecodes.EV_ABS:

                # RT → FORWARD
                if event.code == ecodes.ABS_RZ:
                    self.forward = 1 if event.value < 0 else 0

                # LT → REVERSE
                elif event.code == ecodes.ABS_Z:
                    self.reverse = 1 if event.value < 0 else 0

                # STEERING
                elif event.code == ecodes.ABS_RX:
                    if event.value < -10000:
                        self.steering = -0.5
                    elif event.value > 10000:
                        self.steering = 0.5
                    else:
                        self.steering = 0.0

            elif event.type == ecodes.EV_KEY:

                # RB DEADMAN
                if event.code == ecodes.BTN_TR:
                    self.deadman = event.value

    def publish_cmd(self):
        cmd = Twist()

        if self.deadman == 0:
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0

        else:
            # DIGITAL THROTTLE
            if self.forward == 1:
                cmd.linear.x = 0.6   # forward speed
            elif self.reverse == 1:
                cmd.linear.x = -0.6  # reverse
            else:
                cmd.linear.x = 0.0

            cmd.angular.z = self.steering

        self.cmd_pub.publish(cmd)

        self.get_logger().info(
            f"F:{self.forward} R:{self.reverse} | LIN:{cmd.linear.x:.2f} ANG:{cmd.angular.z:.2f}"
        )


def main():
    rclpy.init()
    node = EvdevJoystick()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
