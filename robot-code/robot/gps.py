import math
import numpy as np


class Gps:

    def __init__(self, hardware, time_step, gyro):
        self.hardware = hardware
        self.time_step = time_step
        self.gyro = gyro

        # Hardware
        self.gps = self.hardware.getDevice("gps")
        self.gps.enable(self.time_step)

        # Variables
        self.initial = [0, 0]
        self.last = [0, 0]
        self.front = [0, 0]

    def update(self):
        # Atualiza o GPS normalizado para o lado certo
        self.last = [self.gps.getValues()[0] - self.initial[0], -self.gps.getValues()[2] + self.initial[2]]
        self.front = [self.last[0] + 0.03284 * math.cos(self.gyro.last), self.last[1] + 0.03284 * math.sin(self.gyro.last)]
        return 
    
    def calibrate(self):
        # Get initial GPS
        self.initial = self.gps.getValues()