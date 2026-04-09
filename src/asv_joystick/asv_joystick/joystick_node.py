import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy

class ASVJoystick(Node):

    def __init__(self):
        super().__init__('asv_joystick_node')

        self.sub = self.create_subscription(Joy, '/joy', self.callback, 10)
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.get_logger().info("🚀 Joystick Ready (JOY MODE)")

    def callback(self, msg):

        twist = Twist()

        # -------- AXES --------
        left_y = msg.axes[7]   # forward/back
        right_x = msg.axes[3]  # turning

        # normalize already [-1,1] from joy_node
        twist.linear.x = left_y
        twist.angular.z = right_x

        # -------- DEADMAN (RB example) --------
        deadman = msg.buttons[5]  # adjust if needed

        if deadman:
            self.pub.publish(twist)
        else:
            self.pub.publish(Twist())  # stop

def main():
    rclpy.init()
    node = ASVJoystick()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
