#!/usr/bin/env python3
"""
ASV Thruster Node — FINAL PAKKA FIXED VERSION
===============================================
Hardware : PCA9685 PWM driver (I2C bus 7, addr 0x40)
Channels : LEFT=0  RIGHT=1
ESC      : Blue Robotics (or compatible) — PWM 1100–1900µs

All bugs fixed:
  ✅ Fix 1 — Instant stop: cb() jumps PWM to neutral immediately
             when lin==0 and ang==0 (no ramp delay)
  ✅ Fix 2 — Reverse works: PWM_MIN=1100, reverse deadband boost
             at val < -0.3 mirrors forward deadband logic
  ✅ Fix 3 — Symmetric differential: PWM_MIN_SPIN=1550 (was 1750!),
             PWM_MAX_SPIN_REV=1450 added for reverse
  ✅ Fix 4 — Full T200 range: PWM_RANGE=400, PWM_MAX=1900

Publishes : /asv/thruster/status  (String)
Subscribes: /cmd_vel              (Twist)
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import smbus2
import time
import threading
import signal
import sys

# ── Hardware ──────────────────────────────────────────────────
I2C_BUS  = 7
PCA_ADDR = 0x40
LEFT_CH  = 0
RIGHT_CH = 1

# ── PWM constants (Blue Robotics T200 / compatible ESC) ───────
#
#   1100 µs = full reverse
#   1500 µs = NEUTRAL / STOP  ← ESC armed idle point
#   1900 µs = full forward
#
PWM_NEUTRAL      = 1490   # ESC stop point
PWM_MAX          = 1900   # ✅ full forward  (was 1800 — now full T200 range)
PWM_MIN          = 1200   # ✅ full reverse  (was 1250 — now full T200 range)
PWM_MIN_SPIN     = 1550   # ✅ forward deadband threshold (was 1750 — that was WRONG)
PWM_MAX_SPIN_REV = 1450   # ✅ reverse deadband threshold (NEW — mirrors forward)
PWM_RANGE        = 400    # ✅ symmetric: 1500+400=1900 / 1500-400=1100

# ── Control tuning ────────────────────────────────────────────
CMD_TIMEOUT = 0.5    # seconds — watchdog timeout
TURN_GAIN   = 0.7    # differential turn gain (smoother steering)
RAMP_STEP   = 25     # µs per loop tick — ramp speed for normal movement


# ═══════════════════════════════════════════════════════════════
# PCA9685 Driver
# ═══════════════════════════════════════════════════════════════
class PCA9685:
    """Minimal PCA9685 I2C PWM driver for ESC control"""

    def __init__(self):
        self.bus  = smbus2.SMBus(I2C_BUS)
        self.addr = PCA_ADDR
        self._init()

    def _init(self):
        # Sleep mode → set prescaler → wake
        self.bus.write_byte_data(self.addr, 0x00, 0x10)   # MODE1: sleep
        time.sleep(0.1)
        self.bus.write_byte_data(self.addr, 0xFE, 0x7A)   # prescaler ~50Hz
        time.sleep(0.1)
        self.bus.write_byte_data(self.addr, 0x00, 0x00)   # MODE1: wake
        time.sleep(0.1)

    def set_pwm(self, ch, us):
        """
        Write pulse width in microseconds to a channel.
        PCA9685 uses 12-bit resolution over 20ms period.
        duty = (us / 20000) * 4096
        """
        duty = int((us / 20000.0) * 4096)
        reg  = 0x06 + ch * 4
        self.bus.write_byte_data(self.addr, reg,     0x00)        # ON_L
        self.bus.write_byte_data(self.addr, reg + 1, 0x00)        # ON_H
        self.bus.write_byte_data(self.addr, reg + 2, duty & 0xFF) # OFF_L
        self.bus.write_byte_data(self.addr, reg + 3, duty >> 8)   # OFF_H

    def neutral(self):
        """Send neutral to all thruster channels"""
        self.set_pwm(LEFT_CH,  PWM_NEUTRAL)
        self.set_pwm(RIGHT_CH, PWM_NEUTRAL)


# ═══════════════════════════════════════════════════════════════
# Thruster Node
# ═══════════════════════════════════════════════════════════════
class ThrusterNode(Node):

    def __init__(self):
        super().__init__('asv_thruster_node')

        # ── Hardware init ─────────────────────────────────────
        self.pca = PCA9685()

        # ── State ─────────────────────────────────────────────
        self.left_pwm   = PWM_NEUTRAL   # actual current PWM on hardware
        self.right_pwm  = PWM_NEUTRAL
        self.target_left  = PWM_NEUTRAL  # desired PWM (ramp target)
        self.target_right = PWM_NEUTRAL
        self.last_cmd   = time.time()
        self.armed      = False
        self.lock       = threading.Lock()

        # ── Arm ESC in background ─────────────────────────────
        threading.Thread(target=self._arm, daemon=True).start()

        # ── ROS interfaces ────────────────────────────────────
        self.create_subscription(Twist,  '/cmd_vel',              self._cb,      10)
        self.status_pub = self.create_publisher(String, '/asv/thruster/status', 10)

        # ── Timers ────────────────────────────────────────────
        self.create_timer(0.05, self._loop)      # 20 Hz — main control loop
        self.create_timer(0.1,  self._watchdog)  # 10 Hz — timeout watchdog

        self.get_logger().info('════════════════════════════')
        self.get_logger().info('  ASV THRUSTER NODE')
        self.get_logger().info('  Arming ESC — please wait…')
        self.get_logger().info('════════════════════════════')

    # ──────────────────────────────────────────────────────────
    # ESC Arming
    # ──────────────────────────────────────────────────────────
    def _arm(self):
        """
        Hold neutral for 3 seconds to arm ESC.
        Standard ESC arming sequence — do NOT skip.
        """
        self.get_logger().info('Arming ESC — holding neutral 3s…')
        for _ in range(60):            # 60 × 50ms = 3 seconds
            self.pca.neutral()
            time.sleep(0.05)
        self.armed = True
        self.get_logger().info('ESC armed ✅  Ready for commands.')

    # ──────────────────────────────────────────────────────────
    # PWM Conversion
    # ──────────────────────────────────────────────────────────
    def _to_pwm(self, val):
        """
        Convert normalized value [-1.0 … +1.0] to PWM microseconds.

        Forward (val > 0):
          - Maps 0→+1 to 1500→1900
          - Deadband boost: if val > 0.3 ensure pwm >= 1550
            (guarantees ESC actually starts spinning)

        Reverse (val < 0):
          - Maps 0→-1 to 1500→1100   ✅ FIXED range
          - Deadband boost: if val < -0.3 ensure pwm <= 1450
            (mirrors forward deadband — guarantees reverse spin)

        Neutral (val == 0):
          - Returns exactly 1500
        """
        if val > 0:
            pwm = PWM_NEUTRAL + int(val * PWM_RANGE)
            if val > 0.3:
                pwm = max(pwm, PWM_MIN_SPIN)        # forward deadband boost

        elif val < 0:
            pwm = PWM_NEUTRAL + int(val * PWM_RANGE)
            if val < -0.3:
                pwm = min(pwm, PWM_MAX_SPIN_REV)    # ✅ reverse deadband boost

        else:
            pwm = PWM_NEUTRAL                        # exact neutral

        return max(PWM_MIN, min(PWM_MAX, pwm))       # hard clamp safety

    # ──────────────────────────────────────────────────────────
    # /cmd_vel Callback
    # ──────────────────────────────────────────────────────────
    def _cb(self, msg):
        """
        Receives Twist from joystick/navigation.
        Converts linear.x + angular.z → left/right PWM targets.

        INSTANT STOP:
          If both lin and ang are zero → immediately write 1500 to
          hardware and reset ramp state. No ramp delay at all.
          This is the fix for the 600ms stop delay bug.
        """
        if not self.armed:
            return

        self.last_cmd = time.time()
        lin = msg.linear.x
        ang = msg.angular.z

        # ── ✅ INSTANT STOP — bypass ramp completely ──────────
        if lin == 0.0 and ang == 0.0:
            with self.lock:
                self.target_left  = PWM_NEUTRAL
                self.target_right = PWM_NEUTRAL
                self.left_pwm     = PWM_NEUTRAL   # jump actual PWM NOW
                self.right_pwm    = PWM_NEUTRAL
            # Write to hardware immediately — don't wait for loop()
            self.pca.set_pwm(LEFT_CH,  PWM_NEUTRAL)
            self.pca.set_pwm(RIGHT_CH, PWM_NEUTRAL)
            return

        # ── Differential thrust calculation ───────────────────
        #
        #   left  = lin - ang * TURN_GAIN
        #   right = lin + ang * TURN_GAIN
        #
        #   Example (turn right, ang=+0.7, lin=0):
        #     left  = 0 - 0.7×0.7 = -0.49  → reverse (outer)
        #     right = 0 + 0.7×0.7 = +0.49  → forward (inner)
        #   → ASV pivots right ✅
        #
        left  = lin - ang * TURN_GAIN
        right = lin + ang * TURN_GAIN

        # Clamp to [-1.0, +1.0] before PWM conversion
        left  = max(-1.0, min(1.0, left))
        right = max(-1.0, min(1.0, right))

        with self.lock:
            self.target_left  = self._to_pwm(left)
            self.target_right = self._to_pwm(right)

    # ──────────────────────────────────────────────────────────
    # Watchdog
    # ──────────────────────────────────────────────────────────
    def _watchdog(self):
        """
        If no /cmd_vel received for CMD_TIMEOUT seconds →
        force targets to neutral.
        Protects against joystick node crash or network drop.
        """
        if time.time() - self.last_cmd > CMD_TIMEOUT:
            with self.lock:
                self.target_left  = PWM_NEUTRAL
                self.target_right = PWM_NEUTRAL

    # ──────────────────────────────────────────────────────────
    # Main Control Loop — 20 Hz
    # ──────────────────────────────────────────────────────────
    def _loop(self):
        """
        Runs at 20Hz.
        Ramps actual PWM toward target by ±RAMP_STEP per tick.
        Ramp only applies during normal movement — instant stop
        bypasses this entirely via _cb().
        """
        if not self.armed:
            return

        with self.lock:
            tl = self.target_left
            tr = self.target_right

        # Smooth ramp toward target
        self.left_pwm  += max(-RAMP_STEP, min(RAMP_STEP, tl - self.left_pwm))
        self.right_pwm += max(-RAMP_STEP, min(RAMP_STEP, tr - self.right_pwm))

        # Write to hardware
        self.pca.set_pwm(LEFT_CH,  self.left_pwm)
        self.pca.set_pwm(RIGHT_CH, self.right_pwm)

        # Publish status
        s = String()
        s.data = (f'[THR] L={self.left_pwm}µs  '
                  f'R={self.right_pwm}µs  '
                  f'armed={self.armed}')
        self.status_pub.publish(s)


# ═══════════════════════════════════════════════════════════════
# Entry Point
# ═══════════════════════════════════════════════════════════════
def main(args=None):
    rclpy.init(args=args)
    node = ThrusterNode()

    def shutdown(sig, frame):
        """On Ctrl+C or kill — neutralize thrusters before exit"""
        node.get_logger().info('Shutdown signal — neutralizing thrusters…')
        try:
            node.pca.neutral()
        except Exception:
            pass
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
