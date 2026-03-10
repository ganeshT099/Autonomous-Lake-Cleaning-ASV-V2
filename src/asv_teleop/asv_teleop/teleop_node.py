#!/usr/bin/env python3
"""
ASV Keyboard Teleop Node
Publishes: /cmd_vel (geometry_msgs/Twist)

Controls:
  W = Forward
  S = Backward
  A = Turn Left
  D = Turn Right
  Q = Forward + Left
  E = Forward + Right
  SPACE = Emergency Stop
  X = Quit
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import sys
import tty
import termios
import threading

BANNER = """
╔══════════════════════════════════════╗
║     ASV Keyboard Teleop Control      ║
╠══════════════════════════════════════╣
║  W = Forward      S = Backward       ║
║  A = Turn Left    D = Turn Right     ║
║  Q = Fwd + Left   E = Fwd + Right    ║
║  SPACE = EMERGENCY STOP              ║
║  X = Quit                            ║
╚══════════════════════════════════════╝
"""

# key → (linear_x, angular_z)
KEY_BINDINGS = {
    'w': ( 1.0,  0.0),   # forward
    's': (-1.0,  0.0),   # backward
    'a': ( 0.0,  1.0),   # turn left
    'd': ( 0.0, -1.0),   # turn right
    'q': ( 1.0,  1.0),   # forward + left
    'e': ( 1.0, -1.0),   # forward + right
    ' ': ( 0.0,  0.0),   # stop
}

class TeleopNode(Node):
    def __init__(self):
        super().__init__('asv_teleop_node')

        self.declare_parameter('max_linear',   0.5)
        self.declare_parameter('max_angular',  1.0)
        self.declare_parameter('publish_hz',   10.0)

        self.max_linear  = self.get_parameter('max_linear').value
        self.max_angular = self.get_parameter('max_angular').value
        self.pub_hz      = self.get_parameter('publish_hz').value

        self.cmd_pub    = self.create_publisher(Twist,  '/cmd_vel',          10)
        self.status_pub = self.create_publisher(String, '/asv/teleop/status', 10)

        self.linear  = 0.0
        self.angular = 0.0
        self.running = True
        self.lock    = threading.Lock()

        # Publish timer
        self.timer = self.create_timer(1.0 / self.pub_hz, self._publish)

        # Keyboard thread
        self.key_thread = threading.Thread(target=self._key_loop, daemon=True)
        self.key_thread.start()

        print(BANNER)
        self.get_logger().info('Keyboard teleop started — use W/A/S/D to control')

    def _get_key(self):
        fd = sys.stdin.fileno()
        old = termios.tcgetattr(fd)
        try:
            tty.setraw(fd)
            key = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old)
        return key

    def _key_loop(self):
        while self.running:
            key = self._get_key().lower()

            if key == 'x':
                self.get_logger().info('Quit — stopping thrusters')
                with self.lock:
                    self.linear  = 0.0
                    self.angular = 0.0
                self.running = False
                rclpy.shutdown()
                break

            if key in KEY_BINDINGS:
                lin, ang = KEY_BINDINGS[key]
                with self.lock:
                    self.linear  = lin * self.max_linear
                    self.angular = ang * self.max_angular
                if key == ' ':
                    print('\n🛑 EMERGENCY STOP!')
                else:
                    labels = {
                        'w':'FORWARD','s':'BACKWARD','a':'LEFT',
                        'd':'RIGHT','q':'FWD+LEFT','e':'FWD+RIGHT'
                    }
                    print(f'\r→ {labels.get(key,"?")} | '
                          f'linear={self.linear:.2f} angular={self.angular:.2f}   ', end='')
            else:
                # Unknown key — stop
                with self.lock:
                    self.linear  = 0.0
                    self.angular = 0.0

    def _publish(self):
        with self.lock:
            lin = self.linear
            ang = self.angular

        msg = Twist()
        msg.linear.x  = lin
        msg.angular.z = ang
        self.cmd_pub.publish(msg)

        s = String()
        s.data = f'[TELEOP] linear={lin:.2f} angular={ang:.2f}'
        self.status_pub.publish(s)

def main(args=None):
    rclpy.init(args=args)
    node = TeleopNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()

if __name__ == '__main__':
    main()
