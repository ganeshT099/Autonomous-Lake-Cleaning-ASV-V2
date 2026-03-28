#!/usr/bin/env python3
"""
ASV Joystick Node - FINAL PAKKA VERSION
=========================================
Direct evdev - no joy node needed!
Auto device detection!

Hardware: Cosmic Byte Stellaris (shows as Xbox360)

Confirmed Mapping:
  Left stick Y  = axis 1  (forward/back)
  Right stick X = axis 3  (turn)
  RB            = 311     (deadman)
  LB            = 310     (boost)
  B             = 305     (estop)
  HOME          = 316     (clear estop)

Publishes:
  /cmd_vel             → Twist
  /asv/mode            → String
  /asv/joystick/status → String
"""

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

# ── Cosmic Byte evdev button codes ───────────────────────────
BTN_A     = 304
BTN_B     = 305   # Emergency stop
BTN_X     = 307
BTN_Y     = 308
BTN_LB    = 310   # Speed boost
BTN_RB    = 311   # Deadman
BTN_BACK  = 314   # Manual mode
BTN_HOME  = 316   # Clear estop

# ── Axis codes ────────────────────────────────────────────────
ABS_LEFT_X  = 0
ABS_LEFT_Y  = 1   # Forward/back
ABS_LT      = 2
ABS_RIGHT_X = 3   # Turn
ABS_RIGHT_Y = 4
ABS_RT      = 5

# ── Speed config ──────────────────────────────────────────────
MAX_LINEAR  = 1.0
MAX_ANGULAR = 1.0
BOOST_MULT  = 1.2   # Safe boost
DEADZONE    = 0.15  # Larger for water stability
SMOOTH      = 0.3   # Smoothing factor (0.3 = responsive)


def find_controller():
    """Auto detect controller device"""
    for path in evdev.list_devices():
        try:
            d = evdev.InputDevice(path)
            if any(name in d.name for name in
                   ['Xbox', 'X-Box', 'Cosmic', 'gamepad', 'Gamepad']):
                print(f'Controller found: {d.name} at {path}')
                return d
        except Exception:
            continue
    return None


class JoystickNode(Node):
    def __init__(self):
        super().__init__('asv_joystick_node')

        # ── State ─────────────────────────────────────────────
        self.linear      = 0.0
        self.angular     = 0.0
        self.raw_linear  = 0.0
        self.raw_angular = 0.0
        self.deadman     = False
        self.boost       = False
        self.estop       = False
        self.mode        = 'MANUAL'
        self.last_joy    = time.time()
        self.axis_max    = {}
        self.lock        = threading.Lock()

        # ── Find controller ───────────────────────────────────
        self.controller = find_controller()
        if self.controller is None:
            self.get_logger().error('Controller not found!')
        else:
            self.get_logger().info(
                f'Controller: {self.controller.name} ✅')
            caps = self.controller.capabilities()
            for code, absinfo in caps.get(ecodes.EV_ABS, []):
                self.axis_max[code] = (absinfo.min, absinfo.max)

        # ── Publishers ────────────────────────────────────────
        self.cmd_pub    = self.create_publisher(
            Twist,  '/cmd_vel',             10)
        self.mode_pub   = self.create_publisher(
            String, '/asv/mode',            10)
        self.status_pub = self.create_publisher(
            String, '/asv/joystick/status', 10)

        # ── Controller read thread ────────────────────────────
        t = threading.Thread(target=self._read_loop)
        t.daemon = True
        t.start()

        # ── Timers ────────────────────────────────────────────
        self.create_timer(0.05, self._publish)   # 20Hz
        self.create_timer(0.1,  self._watchdog)  # 10Hz

        self.get_logger().info('════════════════════════════')
        self.get_logger().info('  ASV JOYSTICK CONTROL')
        self.get_logger().info('  Left stick = Forward/Back')
        self.get_logger().info('  Right stick = Turn')
        self.get_logger().info('  RB  = Deadman hold!')
        self.get_logger().info('  LB  = Boost')
        self.get_logger().info('  B   = Emergency stop')
        self.get_logger().info('  HOME= Clear estop')
        self.get_logger().info('════════════════════════════')

    def _normalize(self, code, value):
        if code in self.axis_max:
            mn, mx = self.axis_max[code]
            if mx != mn:
                return (2.0 * (value - mn) / (mx - mn)) - 1.0
        return value / 32767.0

    def _deadzone(self, value):
        if abs(value) < DEADZONE:
            return 0.0
        return value

    def _read_loop(self):
        if self.controller is None:
            return
        self.get_logger().info('Controller reading started!')
        for event in self.controller.read_loop():
            self.last_joy = time.time()
            with self.lock:
                # ── Axis ──────────────────────────────────────
                if event.type == ecodes.EV_ABS:
                    val = self._normalize(event.code, event.value)
                    val = self._deadzone(val)
                    if event.code == ABS_LEFT_Y:
                        self.raw_linear = -val
                    elif event.code == ABS_RIGHT_X:
                        self.raw_angular = -val

                # ── Buttons ───────────────────────────────────
                elif event.type == ecodes.EV_KEY:
                    # RB deadman
                    if event.code == BTN_RB:
                        self.deadman = bool(event.value)
                        if not self.deadman:
                            self.raw_linear  = 0.0
                            self.raw_angular = 0.0
                            self.linear      = 0.0
                            self.angular     = 0.0
                        self.get_logger().info(
                            f'Deadman: {self.deadman}')

                    # LB boost
                    elif event.code == BTN_LB:
                        self.boost = bool(event.value)

                    # B estop
                    elif event.code == BTN_B and event.value == 1:
                        self.estop       = True
                        self.raw_linear  = 0.0
                        self.raw_angular = 0.0
                        self.linear      = 0.0
                        self.angular     = 0.0
                        self.get_logger().warn('🛑 EMERGENCY STOP!')

                    # HOME clear estop
                    elif event.code == BTN_HOME and event.value == 1:
                        self.estop = False
                        self.get_logger().info('✅ Estop cleared!')

                    # BACK manual mode
                    elif event.code == BTN_BACK and event.value == 1:
                        self.mode  = 'MANUAL'
                        self.estop = False
                        self.get_logger().info('→ MANUAL mode')
                        m = String()
                        m.data = 'MANUAL'
                        self.mode_pub.publish(m)

    def _watchdog(self):
        """Stop everything if no input for 1 second"""
        if time.time() - self.last_joy > 1.0:
            with self.lock:
                self.deadman     = False
                self.raw_linear  = 0.0
                self.raw_angular = 0.0
                self.linear      = 0.0
                self.angular     = 0.0

    def _publish(self):
        with self.lock:
            estop      = self.estop
            deadman    = self.deadman
            raw_lin    = self.raw_linear
            raw_ang    = self.raw_angular
            boost      = self.boost
            mode       = self.mode

        cmd = Twist()

        if not estop and deadman and mode == 'MANUAL':
            # Smoothing filter
            self.linear  = (1 - SMOOTH) * self.linear  + SMOOTH * raw_lin
            self.angular = (1 - SMOOTH) * self.angular + SMOOTH * raw_ang

            # Apply boost
            mult = BOOST_MULT if boost else 1.0
            lin  = self.linear  * MAX_LINEAR  * mult
            ang  = self.angular * MAX_ANGULAR * mult

            # Clamp
            lin = max(-MAX_LINEAR  * BOOST_MULT,
                  min( MAX_LINEAR  * BOOST_MULT, lin))
            ang = max(-MAX_ANGULAR * BOOST_MULT,
                  min( MAX_ANGULAR * BOOST_MULT, ang))

            cmd.linear.x  = lin
            cmd.angular.z = ang
        else:
            # Immediate stop
            self.linear  = 0.0
            self.angular = 0.0

        self.cmd_pub.publish(cmd)

        # Status
        s = String()
        if estop:
            s.data = '[JOY] 🛑 ESTOP — Press HOME to clear'
        elif not deadman:
            s.data = '[JOY] Hold RB to move!'
        elif mode == 'AUTO':
            s.data = '[JOY] AUTO mode'
        else:
            s.data = (f'[JOY] lin={cmd.linear.x:.2f} '
                      f'ang={cmd.angular.z:.2f} '
                      f'boost={boost}')
        self.status_pub.publish(s)


def main(args=None):
    rclpy.init(args=args)
    node = JoystickNode()

    def shutdown(sig, frame):
        print('\nShutting down...')
        cmd = Twist()
        node.cmd_pub.publish(cmd)
        try:
            node.destroy_node()
            rclpy.shutdown()
        except Exception:
            pass
        sys.exit(0)

    signal.signal(signal.SIGINT,  shutdown)
    signal.signal(signal.SIGTERM, shutdown)

    try:
        rclpy.spin(node)
    except SystemExit:
        pass


if __name__ == '__main__':
    main()
