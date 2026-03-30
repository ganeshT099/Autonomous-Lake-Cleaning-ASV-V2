#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import evdev
from evdev import ecodes
import threading
import time
import signal
import sys

# Config tuned for water
MAX_LINEAR  = 0.6
MAX_ANGULAR = 0.6
BOOST_MULT  = 1.1
DEADZONE    = 0.2
SMOOTH      = 0.3


def find_controller():
    for path in evdev.list_devices():
        d = evdev.InputDevice(path)
        if 'Xbox' in d.name or 'Cosmic' in d.name:
            print(f'Controller: {d.name}')
            return d
    return None


class Joy(Node):
    def __init__(self):
        super().__init__('joy_node')

        self.dev = find_controller()
        self.lin = 0
        self.ang = 0
        self.rlin = 0
        self.rang = 0
        self.deadman = False
        self.boost = False
        self.estop = False
        self.lock = threading.Lock()

        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)

        threading.Thread(target=self.read, daemon=True).start()
        self.create_timer(0.05, self.loop)

    def norm(self, v):
        return v / 32767.0

    def dz(self, v):
        return 0 if abs(v) < DEADZONE else v

    def read(self):
        for e in self.dev.read_loop():
            with self.lock:
                if e.type == ecodes.EV_ABS:
                    if e.code == 1:
                        self.rlin = -self.dz(self.norm(e.value))
                    elif e.code == 3:
                        self.rang = -self.dz(self.norm(e.value))

                elif e.type == ecodes.EV_KEY:
                    if e.code == 311:
                        self.deadman = bool(e.value)
                    elif e.code == 310:
                        self.boost = bool(e.value)
                    elif e.code == 305 and e.value:
                        self.estop = True
                    elif e.code == 316 and e.value:
                        self.estop = False

    def loop(self):
        cmd = Twist()

        if self.deadman and not self.estop:
            # smoothing
            self.lin = (1 - SMOOTH) * self.lin + SMOOTH * self.rlin
            self.ang = (1 - SMOOTH) * self.ang + SMOOTH * self.rang

            # cubic scaling (⭐ VERY IMPORTANT)
            lin = (self.lin ** 3) * MAX_LINEAR
            ang = (self.ang ** 3) * MAX_ANGULAR

            if self.boost:
                lin *= BOOST_MULT
                ang *= BOOST_MULT

            cmd.linear.x = lin
            cmd.angular.z = ang
        else:
            self.lin = 0
            self.ang = 0

        self.pub.publish(cmd)


def main():
    rclpy.init()
    node = Joy()
    rclpy.spin(node)


if __name__ == '__main__':
    main()
