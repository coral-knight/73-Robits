import math
import numpy as np


class Gyro:

    def __init__(self, hardware, time_step):
        self.hardware = hardware
        self.time_step = time_step

        # Hardware
        self.gyro = self.hardware.getDevice("gyro")
        self.gyro.enable(self.time_step)

        # Variables
        self.last = 0 

    def update(self):
        ''' 
        Atualiza o Gyro normalizado
        '''
        self.last = self.last + self.gyro.getValues()[1]*self.time_step*0.001
        if self.last > math.pi:
            self.last -= 2*math.pi
        if self.last < -math.pi:
            self.last += 2*math.pi
        return
    
    def calibrate(self, last_gps):
        '''
        Sets the initial Gyro values
        '''
        self.last = math.atan2(last_gps[1] , last_gps[0])
        return