from machine import I2C, Pin, SoftI2C
import time
import math

class QMC5883L:
    def __init__(self, i2c, address=0xd):  # L'indirizzo predefinito per QMC5883
        self.i2c = i2c
        self.address = address
        self.initialize()

    def initialize(self):
        # Set the control registers
        self.i2c.writeto_mem(self.address, 0x0B, bytes([0x01]))  # Software reset
        time.sleep(0.1)
        self.i2c.writeto_mem(self.address, 0x09, bytes([0x1D]))  # Continuous measurement mode, 200Hz, 8G range
        self.i2c.writeto_mem(self.address, 0x0A, bytes([0x00]))  # Set to default

    def read_raw_data(self, reg):
        data = self.i2c.readfrom_mem(self.address, reg, 6)
        x = int.from_bytes(data[0:2], 'little')
        y = int.from_bytes(data[2:4], 'little')
        z = int.from_bytes(data[4:6], 'little')
        if x > 32767: x -= 65536
        if y > 32767: y -= 65536
        if z > 32767: z -= 65536
        return x, y, z

    def read_magnetometer(self):
        x, y, z = self.read_raw_data(0x00)
        return x, y, z

    def heading(self):
        x, y, _ = self.read_magnetometer()
        heading_rad = math.atan2(y, x)
        heading_deg = math.degrees(heading_rad)
        if heading_deg < 0:
            heading_deg += 360
        return heading_deg

    def intensity(self):
        x, y, z = self.read_magnetometer()
        intensity = math.sqrt(x**2 + y**2 + z**2)
        return intensity

    def read(self):
        x, y, z = self.read_magnetometer()
        heading = self.heading()
        intensity = self.intensity()
        return {'x': x, 'y': y, 'z': z, 'heading': heading, 'intensity': intensity}
    

i2c = I2C(0, scl=5, sda=4, freq=400000)
devices = i2c.scan()
dev = QMC5883L(i2c)

dev.read()
