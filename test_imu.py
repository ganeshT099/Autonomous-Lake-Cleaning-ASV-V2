import smbus
import time

bus = smbus.SMBus(7)
addr = 0x4B

print("Testing IMU...")

while True:
    try:
        # Try reading simple register
        data = bus.read_i2c_block_data(addr, 0, 4)
        print("RAW:", data)
    except Exception as e:
        print("I2C ERROR:", e)

    time.sleep(1)

