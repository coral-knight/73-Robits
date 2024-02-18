from hardware import Hardware

from process_lidar import Lidar
from process_camera import Camera

class Sensors:
    def __init__(self):
        self.robot = Hardware(16)

        self.gps = self.robot.getGps()
        self.initial_gps = self.robot.getGps()

        self.gyro = self.robot.getGyro()

    def update(self):
        self.gps = self.robot.getGps()

        self.gyro = self.robot.getGyro()


    
