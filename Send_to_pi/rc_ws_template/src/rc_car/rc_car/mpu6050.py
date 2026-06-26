"""
mpu6050.py — minimal register-level I2C driver for MPU6050 / MPU6500 / MPU9250.

Only the shared accel+gyro path is used (0x3B block), so it works for all three.
(The MPU9250's magnetometer, which would give drift-free yaw, is a separate
AK8963 device — add it later for absolute heading.)

Returns accel in g and gyro in deg/s.
"""

import time

from smbus2 import SMBus

PWR_MGMT_1   = 0x6B
SMPLRT_DIV   = 0x19
CONFIG       = 0x1A
GYRO_CONFIG  = 0x1B
ACCEL_CONFIG = 0x1C
ACCEL_XOUT_H = 0x3B
WHO_AM_I     = 0x75

ACCEL_LSB_PER_G   = 16384.0   # ±2 g full scale
GYRO_LSB_PER_DPS  = 131.0     # ±250 deg/s full scale


class MPU:
    def __init__(self, bus=1, addr=0x68):
        self.addr = addr
        self.bus = SMBus(bus)

    def begin(self):
        self.bus.write_byte_data(self.addr, PWR_MGMT_1, 0x00)    # wake up
        time.sleep(0.05)
        self.bus.write_byte_data(self.addr, SMPLRT_DIV, 0x00)    # no divider
        self.bus.write_byte_data(self.addr, CONFIG, 0x03)        # DLPF ~44 Hz
        self.bus.write_byte_data(self.addr, GYRO_CONFIG, 0x00)   # ±250 dps
        self.bus.write_byte_data(self.addr, ACCEL_CONFIG, 0x00)  # ±2 g
        time.sleep(0.05)

    def who_am_i(self):
        return self.bus.read_byte_data(self.addr, WHO_AM_I)

    @staticmethod
    def _s16(hi, lo):
        v = (hi << 8) | lo
        return v - 65536 if v >= 0x8000 else v

    def read(self):
        """Return (ax, ay, az [g], gx, gy, gz [deg/s])."""
        d = self.bus.read_i2c_block_data(self.addr, ACCEL_XOUT_H, 14)
        ax = self._s16(d[0], d[1]) / ACCEL_LSB_PER_G
        ay = self._s16(d[2], d[3]) / ACCEL_LSB_PER_G
        az = self._s16(d[4], d[5]) / ACCEL_LSB_PER_G
        # d[6], d[7] = temperature (unused)
        gx = self._s16(d[8], d[9]) / GYRO_LSB_PER_DPS
        gy = self._s16(d[10], d[11]) / GYRO_LSB_PER_DPS
        gz = self._s16(d[12], d[13]) / GYRO_LSB_PER_DPS
        return ax, ay, az, gx, gy, gz

    def close(self):
        self.bus.close()
