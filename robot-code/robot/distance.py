import math
import numpy as np


class Distance:

    def __init__(self, hardware, time_step, map, gps, gyro):
        self.hardware = hardware
        self.time_step = time_step
        self.map = map
        self.gps = gps
        self.gyro = gyro

        # Hardware
        self.distance_right = self.hardware.getDevice("distance sensor1")
        self.distance_left = self.hardware.getDevice("distance sensor2")
        self.distance_front = self.hardware.getDevice("distance sensor3")
        self.distance_right.enable(self.time_step)
        self.distance_left.enable(self.time_step)
        self.distance_front.enable(self.time_step)

        # Variables
        self.cvt = 2.7788


    def update(self):
        dist_front = self.distance_front.getValue()
        dist_left = self.distance_left.getValue()
        dist_right = self.distance_right.getValue()

        print("dist front", dist_front)

        if dist_front < 0.8:
            dist_front /= self.cvt
            coord_front = [self.gps.front[0] + dist_front * math.cos(self.gyro.last), self.gps.front[1] + dist_front * math.sin(self.gyro.last)]
            #print("robot front", self.gps.front)
            #print("dist front", dist_front)
            #print("coord front", coord_front)
            self.map.add_obstacle(coord_front)

        if dist_left < 0.8:
            dist_left /= self.cvt
            coord_left = [self.gps.left[0] + dist_left * math.cos(self.gyro.last), self.gps.left[1] + dist_left * math.sin(self.gyro.last)]
            #print("robot left", self.gps.left)
            #print("dist left", dist_left)
            #print("coord left", coord_left)
            self.map.add_obstacle(coord_left)
        
        if dist_right < 0.8:
            dist_right /= self.cvt
            coord_right = [self.gps.right[0] + dist_right * math.cos(self.gyro.last), self.gps.right[1] + dist_right * math.sin(self.gyro.last)]
            #print("robot right", self.gps.right)
            #print("dist right", dist_right)
            #print("coord right", coord_right)
            self.map.add_obstacle(coord_right)

        print("type", self.distance_front.getType())
        #print("lookup table", self.distance_front.getLookupTable())

        return
    
    
    def dist_coords(self, a, b):
        dist = ((a[0]-b[0])**2 + (a[1]-b[1])**2)**0.5
        return dist