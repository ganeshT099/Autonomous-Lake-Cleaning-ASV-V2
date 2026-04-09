import time
import sys
import termios
import tty
import board
import busio
from adafruit_pca9685 import PCA9685

# ================= INIT =================
i2c = busio.I2C(board.SCL, board.SDA)

pca = PCA9685(i2c)
pca.frequency = 50

esc_left = pca.channels[0]
esc_right = pca.channels[1]

# ================= CONFIG =================
MIN = 1000
MAX = 2000

NEUTRAL_LOW = 1760
NEUTRAL_HIGH = 1800
NEUTRAL = (NEUTRAL_LOW + NEUTRAL_HIGH) // 2  # center = 1780

STEP = 40
RAMP_STEP = 15

left_current = NEUTRAL
right_current = NEUTRAL

left_target = NEUTRAL
right_target = NEUTRAL

# ================= FUNCTIONS =================
def set_pulse(channel, us):
    pulse_length = 20000 / 4096
    pulse = int(us / pulse_length)
    channel.duty_cycle = pulse << 4

def apply_deadband(value):
    if NEUTRAL_LOW <= value <= NEUTRAL_HIGH:
        return NEUTRAL
    return value

def ramp(current, target):
    if current < target:
        return min(current + RAMP_STEP, target)
    elif current > target:
        return max(current - RAMP_STEP, target)
    return current

def getch():
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return ch

# ================= ARM =================
print("Arming ESCs...")
for _ in range(100):
    set_pulse(esc_left, NEUTRAL)
    set_pulse(esc_right, NEUTRAL)
    time.sleep(0.02)

print("Controls:")
print("w = forward | s = reverse")
print("a = left | d = right")
print("space = stop | q = quit")

# ================= LOOP =================
try:
    while True:
        key = getch()

        if key == 'w':
            left_target = NEUTRAL_HIGH + STEP
            right_target = NEUTRAL_HIGH + STEP

        elif key == 's':
            left_target = NEUTRAL_LOW - STEP
            right_target = NEUTRAL_LOW - STEP

        elif key == 'a':
            left_target = NEUTRAL_LOW - STEP
            right_target = NEUTRAL_HIGH + STEP

        elif key == 'd':
            left_target = NEUTRAL_HIGH + STEP
            right_target = NEUTRAL_LOW - STEP

        elif key == ' ':
            left_target = NEUTRAL
            right_target = NEUTRAL

        elif key == 'q':
            break

        # Smooth ramp
        left_current = ramp(left_current, left_target)
        right_current = ramp(right_current, right_target)

        # Apply neutral deadband
        left_output = apply_deadband(left_current)
        right_output = apply_deadband(right_current)

        set_pulse(esc_left, left_output)
        set_pulse(esc_right, right_output)

        print(f"L:{left_output}  R:{right_output}")

# ================= SAFE EXIT =================
finally:
    print("Stopping safely...")

    for _ in range(100):
        set_pulse(esc_left, NEUTRAL)
        set_pulse(esc_right, NEUTRAL)
        time.sleep(0.02)

    esc_left.duty_cycle = 0
    esc_right.duty_cycle = 0

    pca.deinit()

    print("Shutdown complete.")
