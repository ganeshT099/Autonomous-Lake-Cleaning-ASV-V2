#!/usr/bin/env python3
"""
ASV Joystick Teleop Node
Subscribes: /joy (sensor_msgs/Joy)
Publishes:  /cmd_vel (geometry_msgs/Twist)
            /asv/joystick/status (std_msgs/String)

Controller Mapping (Xbox/PS5 compatible):
  Left  Stick  UP/DOWN  = Forward / Backward
  Right Stick  LEFT/RIGHT = Turn Left / Right
  RB Button    = Dead man switch (must hold to move)
  LB Button    = Speed boost (hold for full speed)
  B  Button    = Emergency Stop
  Start Button = Enable autonomous mode
  Back Button  = Disable autonomous mode (manual)

Axis mapping (Xbox):
  axes[1] = Left stick Y  (forward/back)
  axes[3] = Right stick X (turn)
  axes[5] = RT (right trigger)
  axes[2] = LT (left trigger)

Button mapping (Xbox):
  buttons[0] = A
  buttons[1] = B  → Emergency stop
  buttons[2] = X
  buttons[3] = Y
  buttons[4] = LB → Speed boost
  buttons[5] = RB → Dead man switch
  buttons[7] = Start → Auto mode
  buttons[6] = Back  → Manual mode
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import time


class JoystickNode(Node):
    def __init__(self):
        super().__init__('asv_joystick_node')

        # ── Parameters ───────────────────────────────────────────
        self.declare_parameter('max_linear',         0.8)
        self.declare_parameter('max_angular',        1.0)
        self.declare_parameter('boost_multiplier',   1.5)
        self.declare_parameter('deadzone',           0.05)
        self.declare_parameter('cmd_timeout',        0.5)
        self.declare_parameter('publish_hz',         20.0)

        # Xbox/PS5 axis mapping
        self.declare_parameter('axis_linear',        1)   # left stick Y
        self.declare_parameter('axis_angular',       3)   # right stick X
        self.declare_parameter('btn_deadman',        5)   # RB
        self.declare_parameter('btn_boost',          4)   # LB
        self.declare_parameter('btn_estop',          1)   # B
        self.declare_parameter('btn_auto',           7)   # Start
        self.declare_parameter('btn_manual',         6)   # Back

        self.max_linear       = self.get_parameter('max_linear').value
        self.max_angular      = self.get_parameter('max_angular').value
        self.boost_mult       = self.get_parameter('boost_multiplier').value
        self.deadzone         = self.get_parameter('deadzone').value
        self.cmd_timeout      = self.get_parameter('cmd_timeout').value
        self.pub_hz           = self.get_parameter('publish_hz').value
        self.axis_linear      = self.get_parameter('axis_linear').value
        self.axis_angular     = self.get_parameter('axis_angular').value
        self.btn_deadman      = self.get_parameter('btn_deadman').value
        self.btn_boost        = self.get_parameter('btn_boost').value
        self.btn_estop        = self.get_parameter('btn_estop').value
        self.btn_auto         = self.get_parameter('btn_auto').value
        self.btn_manual       = self.get_parameter('btn_manual').value

        # ── QoS ──────────────────────────────────────────────────
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=10)

        # ── Subscribers ──────────────────────────────────────────
        self.joy_sub = self.create_subscription(
            Joy, '/joy', self._joy_cb, sensor_qos)

        # ── Publishers ────────────────────────────────────────────
        self.cmd_pub    = self.create_publisher(Twist,  '/cmd_vel',              10)
        self.mode_pub   = self.create_publisher(String, '/asv/mode',             10)
        self.status_pub = self.create_publisher(String, '/asv/joystick/status',  10)

        # ── State ─────────────────────────────────────────────────
        self.linear        = 0.0
        self.angular       = 0.0
        self.estop         = False
        self.mode          = 'MANUAL'   # MANUAL or AUTO
        self.last_joy_time = time.time()

        # ── Timers ────────────────────────────────────────────────
        self.pub_timer      = self.create_timer(1.0 / self.pub_hz, self._publish)
        self.watchdog_timer = self.create_timer(0.1, self._watchdog)

        self.get_logger().info('Joystick node started — hold RB to move!')
        self._print_controls()

    def _print_controls(self):
        self.get_logger().info('═══════════════════════════════════')
        self.get_logger().info('  ASV JOYSTICK CONTROL')
        self.get_logger().info('  Left Stick  = Forward/Backward')
        self.get_logger().info('  Right Stick = Turn Left/Right')
        self.get_logger().info('  RB = Dead man switch (hold!)')
        self.get_logger().info('  LB = Speed boost')
        self.get_logger().info('  B  = Emergency Stop')
        self.get_logger().info('  Start = Auto mode')
        self.get_logger().info('  Back  = Manual mode')
        self.get_logger().info('═══════════════════════════════════')

    def _apply_deadzone(self, value):
        if abs(value) < self.deadzone:
            return 0.0
        return value

    def _joy_cb(self, msg):
        self.last_joy_time = time.time()

        axes    = msg.axes
        buttons = msg.buttons

        # ── Emergency Stop ────────────────────────────────────────
        if len(buttons) > self.btn_estop and buttons[self.btn_estop]:
            self.estop = True
            self.linear  = 0.0
            self.angular = 0.0
            self.get_logger().warn('🛑 EMERGENCY STOP!')
            return

        # ── Mode switching ────────────────────────────────────────
        if len(buttons) > self.btn_auto and buttons[self.btn_auto]:
            if self.mode != 'AUTO':
                self.mode = 'AUTO'
                self.get_logger().info('→ AUTO mode enabled')
                m = String()
                m.data = 'AUTO'
                self.mode_pub.publish(m)

        if len(buttons) > self.btn_manual and buttons[self.btn_manual]:
            if self.mode != 'MANUAL':
                self.mode   = 'MANUAL'
                self.estop  = False
                self.get_logger().info('→ MANUAL mode enabled')
                m = String()
                m.data = 'MANUAL'
                self.mode_pub.publish(m)

        # ── Only move in MANUAL mode ──────────────────────────────
        if self.mode != 'MANUAL':
            return

        # ── Dead man switch — must hold RB ────────────────────────
        deadman_held = (len(buttons) > self.btn_deadman and
                        buttons[self.btn_deadman] == 1)
        if not deadman_held:
            self.linear  = 0.0
            self.angular = 0.0
            return

        # ── Clear estop if deadman pressed ───────────────────────
        self.estop = False

        # ── Speed boost ───────────────────────────────────────────
        boost = (len(buttons) > self.btn_boost and
                 buttons[self.btn_boost] == 1)
        speed_mult = self.boost_mult if boost else 1.0

        # ── Read axes ─────────────────────────────────────────────
        lin = 0.0
        ang = 0.0

        if len(axes) > self.axis_linear:
            lin = self._apply_deadzone(axes[self.axis_linear])
        if len(axes) > self.axis_angular:
            ang = self._apply_deadzone(axes[self.axis_angular])

        # Scale to max speeds
        self.linear  = lin * self.max_linear  * speed_mult
        self.angular = ang * self.max_angular * speed_mult

        # Clamp
        self.linear  = max(-self.max_linear  * self.boost_mult,
                       min( self.max_linear  * self.boost_mult, self.linear))
        self.angular = max(-self.max_angular * self.boost_mult,
                       min( self.max_angular * self.boost_mult, self.angular))

    def _watchdog(self):
        """Stop if no joy message received"""
        if time.time() - self.last_joy_time > self.cmd_timeout:
            self.linear  = 0.0
            self.angular = 0.0

    def _publish(self):
        # Publish cmd_vel
        cmd = Twist()
        if not self.estop and self.mode == 'MANUAL':
            cmd.linear.x  = self.linear
            cmd.angular.z = self.angular
        self.cmd_pub.publish(cmd)

        # Status
        s = String()
        if self.estop:
            s.data = '[JOY] 🛑 ESTOP'
        elif self.mode == 'AUTO':
            s.data = '[JOY] AUTO mode — joystick disabled'
        else:
            s.data = (f'[JOY] MANUAL | '
                      f'linear={self.linear:.2f} '
                      f'angular={self.angular:.2f}')
        self.status_pub.publish(s)


def main(args=None):
    rclpy.init(args=args)
    node = JoystickNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
