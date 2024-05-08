import math
import numpy as np


class Camera:

    def __init__(self):
        self.sign_list = []


    def collect(self, last_gps):
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
            if self.dist_coords(last_gps, sign[1]) < closest[1]:
                closest = [i, self.dist_coords(last_gps, sign[1])]
        self.sign_list.pop(closest[0])

        return
    
    
    '''========================================= AUXILIAR FUNCTIONS ==========================================='''


    def dist_coords(self, a, b):
        dist = ((a[0]-b[0])**2 + (a[1]-b[1])**2)**0.5
        return dist