import math
import numpy as np
import struct
import cv2


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
        self.camera_left.setFov(1.5)
        
        self.camera_right = self.hardware.getDevice("camera2")
        self.camera_right.enable(self.time_step*5)
        self.camera_right.setFov(1.5)

        # Variables
        self.left_size = [128, 40]
        self.right_size = [128, 40]
        self.sign_list = []

        self.c_initial_tick = True
        self.c_dir = 0
        self.c_last_tick_gyro = 0
        self.c_total_gyro = 0

        self.c_found = False
        self.c_centered = False
        self.c_far = False
        self.c_identified = False
        self.c_type = 'N'
        self.c_close = False
        self.c_sent = 0
        self.c_timer = 0
        self.c_collect_time = 4500
        self.c_back = False


    def joint_image(self):
        # Get cameras' images and join them into a single one

        img_data_l = self.camera_left.getImage()
        img_data_r = self.camera_right.getImage()
        img_l = np.array(np.frombuffer(img_data_l, np.uint8).reshape((self.camera_left.getHeight(), self.camera_left.getWidth(), 4)))
        img_r = np.array(np.frombuffer(img_data_r, np.uint8).reshape((self.camera_right.getHeight(), self.camera_right.getWidth(), 4)))

        img = np.empty([32,256,4])
        img[0:, 0:128] = img_l[0:, 0:]
        img[0:, 128:256] = img_r[0:, 0:]

        return img

        return 
    

    def is_wall(self, color):
        yes = False

        if abs(color[1]-2*color[0]) < 10 and abs(color[2]-color[1]-(color[1]-color[0])/6) < 3:
            yes = True

        if (color[0] == color[1] and color[1] == color[2]):
            yes = False
        if (color[2]-color[1] > abs(color[1]-color[0])):
            yes = False

        return yes
    

    def delete(self, coords):
        closest = [-1, 1000]
        for i in range(len(self.sign_list)):
            sign = self.sign_list[i]
            if self.dist_coords(coords, sign[1]) < closest[1]:
                closest = [i, self.dist_coords(coords, sign[1])]
        self.sign_list.pop(closest[0])


    def collect(self, navigate, emitter):
        # Controller for all components and actions of the Collect function

        print("=======\nCollect")

        img = self.joint_image()
        
        if self.c_initial_tick:
            # c_dir = 1 if turning right and -1 if turning left
            self.c_dir = 1 if self.lidar.ray_dist(492) > self.lidar.ray_dist(20) else -1 
            self.c_last_tick_gyro = self.gyro.last
            self.c_initial_tick = False

        if not self.c_found and self.c_total_gyro > 2*math.pi:
            self.c_initial_tick = True
            self.c_total_gyro = 0
            print("rodou tudo")
            return False
        
        if not self.c_found and self.is_wall([img.item(16, 128, 2), img.item(16, 128, 1), img.item(16, 128, 0)]):
            navigate.speed(self.c_dir*navigate.turn_velocity, -self.c_dir*navigate.turn_velocity)

            self.c_total_gyro += min(abs(self.gyro.last-self.c_last_tick_gyro), 2*math.pi-abs(self.gyro.last-self.c_last_tick_gyro))
            self.c_last_tick_gyro = self.gyro.last

            self.c_found = False
            self.c_centered = False
            self.c_far = False
            self.c_identified = False
            self.c_type = 'N'
            self.c_close = False
            self.c_sent = 0
            self.c_timer = 0
            self.c_back = False

        elif self.lidar.ray_dist(0) < 0.08:
            print("pixel diferente de parede")
            self.c_found = True

            # Center the victim within cameras' image
            if not self.c_centered:  
                navigate.speed(self.c_dir*1, -self.c_dir*1)
                left_count, right_count = 0, 0
                while not self.is_wall([img.item(16, 128-left_count, 2), img.item(16, 128-left_count, 1), img.item(16, 128-left_count, 0)]): left_count += 1
                while not self.is_wall([img.item(16, 128+right_count, 2), img.item(16, 128+right_count, 1), img.item(16, 128+right_count, 0)]): right_count += 1
                print("centralizando", left_count, right_count)

                if (self.c_dir == 1 and left_count > right_count) or (self.c_dir == -1 and left_count < right_count): 
                    self.c_centered = True

            # Get far to identify
            #elif not self.c_far:  
            #    print("distancia")
            #    navigate.speed(-navigate.turn_velocity, -navigate.turn_velocity)
            #    if self.lidar.ray_dist(0) > 0.07:
            #        print("deu")
            #        self.c_far = True

            # Identify the sign's type
            elif not self.c_identified:  
                navigate.speed(0, 0)
                #self.c_type = self.identify(img)
                self.c_type = 'H'
                self.c_identified = True
                print("identificado", self.c_type)

            # Get closer to send the right sign
            elif not self.c_close:  
                print("aproxima")
                navigate.speed(navigate.turn_velocity, navigate.turn_velocity)
                if self.lidar.ray_dist(0) < 0.05:
                    print("deu")
                    self.c_close = True

            # Send the sign with emitter
            elif self.c_sent <= 0: 
                navigate.speed(0, 0)
                self.c_timer += 1
                print(self.c_timer*self.time_step)

                if self.c_sent != -1 and self.c_timer*self.time_step > 1500:
                    dist = self.lidar.ray_dist(0)
                    sign_coords = [self.gps.last[0] + dist * math.cos(self.gyro.last), self.gps.last[1] + dist * math.sin(self.gyro.last)]
                    sign_type = bytes(self.c_type, "utf-8")
                    print("tempo de mandar")
                    print("coords", dist, sign_coords)

                    a = self.gps.gps.getValues()

                    message = struct.pack("i i c", int((sign_coords[0]+self.gps.initial[0])*100), int((-sign_coords[1]+self.gps.initial[2])*100), sign_type)
                    emitter.send(message)

                    self.delete(sign_coords)
                    self.c_sent = -1
                
                if self.c_timer*self.time_step > 1550: 
                    print("terminou de mandar")
                    self.c_sent = 1

            # Get back to initial position
            elif not self.c_back:  
                print("volta")
                navigate.speed(-navigate.turn_velocity, -navigate.turn_velocity)
                if self.lidar.ray_dist(0) > 0.055:
                    self.c_back = True
                    print("deu")
                    
            else:
                print("volta a rodar")
                navigate.speed(self.c_dir*navigate.turn_velocity, -self.c_dir*navigate.turn_velocity)
                self.c_found = False


        return True
    
    
    '''========================================= AUXILIAR FUNCTIONS ==========================================='''


    def dist_coords(self, a, b):
        dist = ((a[0]-b[0])**2 + (a[1]-b[1])**2)**0.5
        return dist