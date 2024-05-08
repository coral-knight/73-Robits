import math
import numpy as np


class Camera:

    def __init__(self, hardware, time_step, gps):
        self.hardware = hardware
        self.time_step = time_step
        self.gps = gps

        #Centered camera
        self.camera_front = self.hardware.getDevice("camera1")
        self.camera_front.enable(self.time_step*5)
        
        #Increased central angle camera
        self.camera_plus = self.hardware.getDevice("camera2")
        self.camera_plus.enable(self.time_step*5)

        #Decreased central angle camera
        self.camera_sub = self.hardware.getDevice("camera3")
        self.camera_sub.enable(self.time_step*5)

        self.sign_list = []


    def collect(self):
        # Collect sign and delete it from the sign_list

        print("========== tentando coletar ==========")

        ''' 
        
        Collect 

        while last_gyro != initial_collect_gyro:
            rotate (I) (based on the left and right distance)
            if front dist less than *0.08* and mid pixel is not wall:
                rotate until sign is centralized
                identify
                walk front 
                send sign
                walk back 
                rotate until sign is out of the mid pixel (I)
            
        '''

        closest = [-1, 1000]
        for i in range(len(self.sign_list)):
            sign = self.sign_list[i]
            if self.dist_coords(self.gps.last, sign[1]) < closest[1]:
                closest = [i, self.dist_coords(self.gps.last, sign[1])]
        self.sign_list.pop(closest[0])

        return
    
    
    '''========================================= AUXILIAR FUNCTIONS ==========================================='''


    def dist_coords(self, a, b):
        dist = ((a[0]-b[0])**2 + (a[1]-b[1])**2)**0.5
        return dist