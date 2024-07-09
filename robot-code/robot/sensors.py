import math
import numpy as np
import cv2
#from robot.distance import Distance
from robot.lidar import Lidar
from robot.camera import Camera
from robot.gps import Gps
from robot.gyro import Gyro
from robot.emitter import Emitter
from robot.receiver import Receiver

class Sensors:
    def __init__(self, hardware, time_step, map):
        self.hardware = hardware
        self.time_step = time_step
        self.map = map

        self.emitter = Emitter(self.hardware)
        self.receiver = Receiver(self.hardware, self.time_step)

        self.gyro = Gyro(self.hardware, self.time_step)
        self.gps = Gps(self.hardware, self.time_step, self.gyro)
        #self.distance = Distance(self.hardware, self.time_step, self.map, self.gps, self.gyro)
        self.lidar = Lidar(self.hardware, self.time_step, self.map, self.gps, self.gyro)
        self.camera = Camera(self.hardware, self.time_step, self.emitter, self.gps, self.gyro, self.lidar, self.map)


    def update(self, current_tick, turning):
        self.emitter.update()
        self.receiver.update()

        self.gyro.update()
        self.gps.update()
        #self.distance.update()

        if max(abs(self.gyro.last_front), abs(self.gyro.last_side)) < 0.15:
            if current_tick % 5 == 0:
                self.lidar.update(current_tick)

            if current_tick % 10 == 0:
                self.camera.seen(current_tick)

            if current_tick % 20 == 0 and self.camera.c_initial_tick == True and turning == False:
                #self.camera.identify_token(self.camera.joint_image(), "left")
                self.camera.update_token(current_tick)
                self.camera.update_ground()
                #self.camera.print_list()        

        for i in range(-1, 2):
            for j in range(-1, 2):
                self.map.explored([self.gps.last[0]+i*0.02, self.gps.last[1]+j*0.02])

        if current_tick % 20 == 0: self.map.to_detailed_png(current_tick)
        self.map.to_png_seen()

        return

    

    
        