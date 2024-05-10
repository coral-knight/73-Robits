import math
import numpy as np


class Camera:

    def __init__(self, hardware, time_step, gps, gyro, lidar):
        self.hardware = hardware
        self.time_step = time_step
        self.gps = gps
        self.gyro = gyro
        self.lidar = lidar

        # Hardware
        self.camera_left = self.hardware.getDevice("camera1")
        self.camera_left.enable(self.time_step*5)
        
        self.camera_plus = self.hardware.getDevice("camera2")
        self.camera_plus.enable(self.time_step*5)

        self.camera_sub = self.hardware.getDevice("camera3")
        self.camera_sub.enable(self.time_step*5)

        # Variables
        self.sign_list = []

        self.c_initial_tick = True
        self.c_dir = 0
        self.c_initial_gyro = 0

        self.c_centered = False
        self.c_identified = False
        self.c_close = False
        self.c_sent = False
        self.c_far = False
        self.c_next = False


    def image():
        # Get cameras' images and join them into a single one

        return 


    def collect(self, navigate):
        '''
        Controller for all components and actions of the Collect function
        
        if self.initial_tick:
            self.c_dir = self.lidar.ray_dist( [raio para a esquerda] ) > self.lidar.ray_dist( [raio para a direita] ) ? 1 : -1 # 1 if turning left and -1 if turning right
            self.c_initial_gyro = self.gyro.last - dir*0.3
            self.c_initial_tick = False

        if abs(self.gyro.last - self.c_initial_gyro) < 0.25 or 2*math.pi - abs(self.gyro.last - self.c_initial_gyro) < 0.25:
            return False

        if * mid pixel is wall *:
            navigate.speed(dir*navigate.turn_velocity, -dir*navigate.turn_velocity)
        else:
            if not c_centered:
                
            elif not c_identified:
            
            elif not c_close:
            
            elif not c_sent:
            
            elif not c_far:
            
            elif not c_next:
                navigate.speed(dir*navigate.turn_velocity, -dir*navigate.turn_velocity)
                if * mid pixel is wall *:
                    self.c_centered = False
                    self.c_identified = False
                    self.c_close = False
                    self.c_sent = False
                    self.c_far = False
                    self.c_next = True

        '''

        return True


    def collect_2(self, navigate):
        # Collect sign and delete it from the sign_list

        print("========== tentando coletar ==========")

        ''' 
        
        Collect 

        dir = self.lidar.ray_dist( [raio para a esquerda] ) > self.lidar.ray_dist( [raio para a direita] ) ? 1 : -1 # 1 if turning left and -1 if turning right
        initial_gyro = self.gyro.last - dir*0.01
        print("last gyro", self.gyro.last)
        print("dir and initial_gyro", dir, initial_gyro)

        while self.gyro.last != initial_gyro: # Rotate 2*pi in case there's other nearby victims that I am able to collect

            - rotate (I) (based on the left and right distance)
            navigate.speed(dir*navigate.turn_velocity, -dir*navigate.turn_velocity)
            
            if self.lidar.ray_dist( [raio para frente (0 ou 256) ] ) < 0.08 and * mid pixel is not wall *:
                - rotate until sign is centralized
                while [ * dir * sign pixels ] > [ * -dir * sign pixels ]: navigate.speed(dir*navigate.turn_velocity, -dir*navigate.turn_velocity)

                - identify
                self.identify(* image *)

                - walk front 
                while self.lidar.ray_dist(0) > 0.043: navigate.speed(navigate.velocity, navigate.velocity)

                - send sign


                - walk back 
                while self.lidar.ray_dist(0) < 0.059: navigate.speed(-navigate.velocity, -navigate.velocity)

                - rotate until sign is out of the mid pixel (I)
                while * mid pixel is not wall *: navigate.speed(dir*navigate.turn_velocity, -dir*navigate.turn_velocity)
            
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