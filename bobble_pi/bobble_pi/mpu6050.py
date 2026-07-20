"""Minimal I2C driver for the MPU6050 IMU (accelerometer + gyroscope,
no magnetometer). Register map and scale factors are from the InvenSense
MPU-6050 register map datasheet.

Requires smbus2 on the Raspberry Pi:
    pip install smbus2
"""

import math
import struct
import time

try:
    from smbus2 import SMBus
except ImportError:  # smbus2 is only available/needed on the Pi itself
    SMBus = None

PWR_MGMT_1 = 0x6B
SMPLRT_DIV = 0x19
CONFIG = 0x1A
GYRO_CONFIG = 0x1B
ACCEL_CONFIG = 0x1C
ACCEL_XOUT_H = 0x3B
GYRO_XOUT_H = 0x43
WHO_AM_I = 0x75

ACCEL_SCALE_2G = 16384.0  # LSB / g, +/-2g full scale range
GYRO_SCALE_250DPS = 131.0  # LSB / (deg/s), +/-250 deg/s full scale range
GRAVITY_MS2 = 9.80665


class Mpu6050:
    def __init__(self, bus_num=1, address=0x68):
        if SMBus is None:
            raise RuntimeError(
                "smbus2 is not installed. On the Raspberry Pi run: "
                "pip install smbus2 (or sudo apt install python3-smbus2)."
            )
        self.address = address
        self.bus = SMBus(bus_num)

        # The sensor powers up in sleep mode - wake it.
        self.bus.write_byte_data(self.address, PWR_MGMT_1, 0x00)
        time.sleep(0.05)
        # Sample rate = 1kHz / (1 + 4) = 200Hz, DLPF ~42Hz, +/-250dps, +/-2g.
        self.bus.write_byte_data(self.address, SMPLRT_DIV, 0x04)
        self.bus.write_byte_data(self.address, CONFIG, 0x03)
        self.bus.write_byte_data(self.address, GYRO_CONFIG, 0x00)
        self.bus.write_byte_data(self.address, ACCEL_CONFIG, 0x00)

        self.gyro_bias = (0.0, 0.0, 0.0)

    def _read_word(self, reg):
        high = self.bus.read_byte_data(self.address, reg)
        low = self.bus.read_byte_data(self.address, reg + 1)
        value = (high << 8) | low
        return struct.unpack('>h', struct.pack('>H', value))[0]

    def read_raw(self):
        """Returns ((ax, ay, az) in m/s^2, (gx, gy, gz) in rad/s), with no
        bias correction applied."""
        ax = self._read_word(ACCEL_XOUT_H) / ACCEL_SCALE_2G * GRAVITY_MS2
        ay = self._read_word(ACCEL_XOUT_H + 2) / ACCEL_SCALE_2G * GRAVITY_MS2
        az = self._read_word(ACCEL_XOUT_H + 4) / ACCEL_SCALE_2G * GRAVITY_MS2
        gx = math.radians(self._read_word(GYRO_XOUT_H) / GYRO_SCALE_250DPS)
        gy = math.radians(self._read_word(GYRO_XOUT_H + 2) / GYRO_SCALE_250DPS)
        gz = math.radians(self._read_word(GYRO_XOUT_H + 4) / GYRO_SCALE_250DPS)
        return (ax, ay, az), (gx, gy, gz)

    def calibrate_gyro(self, samples=400, delay=0.002):
        """The robot MUST be held still and level while this runs. Averages
        raw gyro readings to estimate the zero-rate bias of each axis."""
        sx = sy = sz = 0.0
        for _ in range(samples):
            _, (gx, gy, gz) = self.read_raw()
            sx += gx
            sy += gy
            sz += gz
            time.sleep(delay)
        self.gyro_bias = (sx / samples, sy / samples, sz / samples)
        return self.gyro_bias

    def read(self):
        """Returns bias-corrected ((ax, ay, az), (gx, gy, gz))."""
        (ax, ay, az), (gx, gy, gz) = self.read_raw()
        bx, by, bz = self.gyro_bias
        return (ax, ay, az), (gx - bx, gy - by, gz - bz)
