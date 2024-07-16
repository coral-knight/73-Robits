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

        self.rot_initial_pos = [0, 0]


    def update(self, current_tick, turning):
        self.emitter.update()
        self.receiver.update()

        self.gyro.update()
        self.gps.update()
        #self.distance.update()

        if max(abs(self.gyro.last_front), abs(self.gyro.last_side)) > 0.007:
            #print("ROTATED", self.gyro.last_front, self.gyro.last_side)
            if self.dist_coords(self.gps.last, self.rot_initial_pos) > 0.03:
                #print("FIXED")
                self.gyro.last_front, self.gyro.last_side = 0, 0
        else: self.rot_initial_pos = self.gps.last 

        if max(abs(self.gyro.last_front), abs(self.gyro.last_side)) < 0.01:
            if current_tick % 10 == 0:
                self.lidar.update(current_tick)

            #if current_tick % 10 == 0:
            #    self.camera.seen(current_tick)

            if current_tick % 10 == 0 and self.camera.c_initial_tick == True and turning == False:
                self.camera.update_token(current_tick)
                if current_tick % 10 == 0: self.camera.update_ground()

        for i in range(-10, 11):
            angle = self.gyro.last + i * math.pi/20
            self.map.explored([self.gps.last[0] + 0.035*math.cos(angle), self.gps.last[1] + 0.035*math.sin(angle)])

        if current_tick % 20 == 0: self.map.to_detailed_png(current_tick)
        self.map.to_png_seen()

        return
    
    
    def dist_coords(self, a, b):
        dist = ((a[0]-b[0])**2 + (a[1]-b[1])**2)**0.5
        return dist
    

    

    
        