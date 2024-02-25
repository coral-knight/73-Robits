
from process_lidar import Lidar
from process_camera import Camera

class Sensors:
    def __init__(self, hardware):
        self.hardware = hardware

        self.gps = self.hardware.getGps()
        self.initial_gps = self.hardware.getGps()

        self.gyro = self.hardware.getGyro()

    def update(self):
        self.gps = self.hardware.getGps()

        self.gyro = self.hardware.getGyro()

    