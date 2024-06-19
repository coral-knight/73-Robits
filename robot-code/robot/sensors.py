import math
from robot.lidar import Lidar
from robot.camera import Camera
from robot.gps import Gps
from robot.gyro import Gyro

class Sensors:
    def __init__(self, hardware, time_step, map):
        self.hardware = hardware
        self.time_step = time_step
        self.map = map

        self.gyro = Gyro(self.hardware, self.time_step)
        self.gps = Gps(self.hardware, self.time_step, self.gyro)
        self.lidar = Lidar(self.hardware, self.time_step, self.map, self.gps, self.gyro)
        self.camera = Camera(self.hardware, self.time_step, self.gps, self.gyro, self.lidar, self.map)


    def update(self, current_tick):
        self.gps.update()
        self.gyro.update()

        if current_tick % 5 == 0:
            self.lidar.update()
            self.camera.seen()
            self.camera.update_ground()

        #if current_tick % 10 == 0:
            #self.camera.update_token()
            #self.camera.update_ground()

        for i in range(-1, 2):
            for j in range(-1, 2):
                self.map.explored([self.gps.last[0]+i*0.02, self.gps.last[1]+j*0.02])

        self.map.to_png_seen()

        return

    

    
        