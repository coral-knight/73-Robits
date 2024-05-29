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
        self.sign_colleted = []

        self.c_initial_tick = True
        self.c_dir = 0
        self.c_last_tick_gyro = 0
        self.c_total_gyro = 0
        self.c_collected = []

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

        img = np.empty([40,256,4])
        img[0:, 0:128] = img_l[0:, 0:]
        img[0:, 128:256] = img_r[0:, 0:]

        return img
    

    def is_wall(self, color):
        yes = False

        if abs(color[1]-2*color[0]) < 10 and abs(color[2]-color[1]-(color[1]-color[0])/6) < 3:
            yes = True

        if (color[0] == color[1] and color[1] == color[2]):
            yes = False
        if (color[2]-color[1] > abs(color[1]-color[0])):
            yes = False

        return yes
    

    def collected(self, coords):
        closest = [-1, 1000]
        for i in range(len(self.sign_list)):
            sign = self.sign_list[i]
            if self.dist_coords(coords, sign[1]) < closest[1]:
                closest = [i, self.dist_coords(coords, sign[1])]

        print("closest", self.sign_list[closest[0]])

        self.sign_colleted.append(self.sign_list[closest[0]])
        self.sign_list.pop(closest[0])

        return


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
            self.c_collected = []
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
            for c in self.c_collected:
                print("diff", c, min(abs(self.gyro.last-c), 2*math.pi-abs(self.gyro.last-c)))
                if not self.c_found and min(abs(self.gyro.last-c), 2*math.pi-abs(self.gyro.last-c)) < 0.3:
                    return True

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

                    self.collected(sign_coords)
                    self.c_collected.append(self.gyro.last)
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
    

    def find(self, image): # find objects in camera
        img = image

        topwall = np.zeros(256, dtype=int) # array of how many times see wall, wall, non-wall
        deltas = [] # victim positions 
        x_right = -1 # x at  extreme right
        x_left = 257 # x at extreme left
        #for all pixels in image: 
        for y in range(1,39): 
            for x in range(0, 256): 
                colour_rgb1 = [img.item(y, x, 2), img.item(y, x, 1), img.item(y, x, 0)] 
                colour_rgb2 = [img.item(y+1, x, 2), img.item(y+1, x, 1), img.item(y+1, x, 0)]
                colour_rgb3 = [img.item(y-1, x, 2), img.item(y-1, x, 1), img.item(y-1, x, 0)]

                if self.is_wall(colour_rgb1) and self.is_wall(colour_rgb2) and not self.is_wall(colour_rgb3): 
                    if topwall[x] == 1: 
                        topwall[x] = y 

                        up_camera = colour_rgb3
                        right_camera = colour_rgb3

                        up_count = 0
                        right_count = 0

                        # enquanto não for parede, up_camera
                        while((y-up_count) > 0 and not self.is_wall(up_camera)):
                            up_count = up_count + 1
                            up_camera = [img.item(y-up_count, x, 2), img.item(y-up_count, x, 1), img.item(y-up_count, x, 0)]
                            #print("going up", (y-up_count), x, up_camera)

                        up_count = int((up_count+1)/2)

                        # enquanto não for parede, right_camera
                        while((x+right_count) < 255 and not self.is_wall(right_camera)):
                            right_count = right_count + 1
                            right_camera = [img.item(y-up_count, x+right_count, 2), img.item(y-up_count, x+right_count, 1), img.item(y-up_count, x+right_count, 0)]
                            #print("going right", (y-up_count), x+right_count, right_camera)

                        if x+right_count-1 - x >= 3 and (x > x_right + 1 or x+right_count-1 < x_left - 1):
                            x_left = x
                            x_right = x+right_count-1
                            #print("vítima", x_left, x_right, y-up_count, len(deltas)) 

                            deltas.append([x_left, x_right])

                    else: 
                        ceiling = [img.item(0, x, 2), img.item(0, x, 1), img.item(0, x, 0)] 
                        if self.is_wall(ceiling):

                            # canto inferior esquerdo da vítima
                            up_camera = colour_rgb3 
                            right_camera = colour_rgb3  

                            up_count = 0 
                            right_count = 0  

                            # enquanto for parede, up_camera
                            while((y-up_count) > 0 and not self.is_wall(up_camera)): 
                                up_count = up_count + 1 
                                up_camera = [img.item(y-up_count, x, 2), img.item(y-up_count, x, 1), img.item(y-up_count, x, 0)] 
                                #print("going up 2", (y-up_count), x, up_camera)

                            up_count = int((up_count+1)/2) 

                            # enquanto for parede, right_camera
                            while((x+right_count) < 255 and not self.is_wall(right_camera)):
                                right_count = right_count + 1 # conta larura
                                right_camera = [img.item(y-up_count, x+right_count, 2), img.item(y-up_count, x+right_count, 1), img.item(y-up_count, x+right_count, 0)] # leva pixel para a direita
                                #print("going right 2", (y-up_count), x+right_count, right_camera)


                            if x+right_count-1 - x >= 3 and (x > x_right + 1 or x+right_count-1 < x_left - 1):
                                x_left = x 
                                x_right = x+right_count-1 
                                #print("vítima2", x_left, x_right, len(deltas)) 
                                
                                deltas.append([x_left, x_right])

                        elif topwall[x] == 0: # como tem pixel de não-parede => topwall = 1 de céu
                            topwall[x] = 1
        return deltas, topwall


    def update(self): 
        #img = np.array(np.frombuffer(img_data, np.uint8).reshape((self.camera_right.getHeight(), self.camera_right.getWidth(), 4)))
        #("img" + "_" + type_camera + ".png", img)

        #print("process cameras =========")

        img = self.joint_image()
        deltas, topwall = self.find(img)

        for d in deltas:
            [x_left, x_right] = d
            
            if abs(topwall[x_right]-topwall[x_left]) < 5 and (topwall[x_right] > 1 or self.is_wall([img.item(0, x_right, 2), img.item(0, x_right, 1), img.item(0, x_right, 0)])):
                #print("aqui", x_left, x_right)
                #cv2.imwrite("possible_victim_" + str(tick_count) + ".png", img)

                #img_angle = math.atan((-((x_left+x_right)/2)+128) * math.tan(3/2) / 128)
                #for image in left and in right
                if ((x_left+x_right)/2) <= 128: 
                    img_angle = math.atan((-((x_left+x_right)/2)+64) * math.tan(1.5/2) / 64) + 0.75
                if ((x_left+x_right)/2) > 128:
                    img_angle = math.atan((-((x_left+x_right)/2-128)+64) * math.tan(1.5/2) / 64) - 0.75
                
                raio = round((math.pi-(img_angle))*256/math.pi)
                raio = (raio+255) % 512

                dist = self.lidar.ray_front_dist(raio)
                #print("dist", dist)
                #print("raio", raio)
                #print("img angle", img_angle)
                #print("ang total", self.gyro.last+img_angle)

                if dist < 0.18:
                    a = self.gps.front[0] + dist * (math.cos(self.gyro.last+img_angle))
                    b = self.gps.front[1] + dist * (math.sin(self.gyro.last+img_angle))

                    #print("coords", a, b)

                    rd = 1

                    if dist < 0.08:
                        rd = 3

                    aux = 2*math.pi/512

                    ae = self.gps.front[0] + self.lidar.ray_front_dist((raio-rd+512)%512) * (math.cos(self.gyro.last+img_angle+rd*aux))
                    be = self.gps.front[1] + self.lidar.ray_front_dist((raio-rd+512)%512) * (math.sin(self.gyro.last+img_angle+rd*aux))
                    ad = self.gps.front[0] + self.lidar.ray_front_dist((raio+rd)%512) * (math.cos(self.gyro.last+img_angle-rd*aux))
                    bd = self.gps.front[1] + self.lidar.ray_front_dist((raio+rd)%512) * (math.sin(self.gyro.last+img_angle-rd*aux))

                    ang = math.atan2(b - self.gps.last[1], a - self.gps.last[0]) 
                    ang_max = math.atan2(be-bd, ae-ad)
                    ang_min = math.atan2(bd-be, ad-ae)
                    
                    ae = self.gps.front[0] + self.lidar.ray_front_dist((raio-6+512)%512) * (math.cos(self.gyro.last+img_angle+6*aux))
                    be = self.gps.front[1] + self.lidar.ray_front_dist((raio-6+512)%512) * (math.sin(self.gyro.last+img_angle+6*aux))
                    ad = self.gps.front[0] + self.lidar.ray_front_dist((raio+6)%512) * (math.cos(self.gyro.last+img_angle-6*aux))
                    bd = self.gps.front[1] + self.lidar.ray_front_dist((raio+6)%512) * (math.sin(self.gyro.last+img_angle-6*aux))

                    if (self.dist_coords([ae, be], [ad, bd]) < 0.05) and (self.dist_coords([ae, be], [a, b]) < 0.03) and (self.dist_coords([a, b], [ad, bd]) < 0.03):
                        # sign = [[x_right-x_left], [pos], [left point pos], [right point pos]]

                        #print("wall angles", ang, ang_min, ang_max)
                        #print(self.dist_coords([ae, be], [ad, bd]))

                        cont = 0
                        vitima_igual = False

                        #print("vai ver done")
                        for v in self.sign_colleted:
                            if (self.dist_coords([a, b], v[1]) < 0.033 and (abs(v[4][0]-ang_min) < math.pi/3 or 2*math.pi-abs(v[4][0]-ang_min) < math.pi/3)):
                                #print("já pegou")
                                vitima_igual = True

                        #print("certo", len(self.sign_list))
                        for v in self.sign_list:
                            if self.dist_coords([a, b], v[1]) < 0.033:
                                #print("vitima perto")
                                if abs(v[4][0]-ang_min) < 0.3 or 2*math.pi-abs(v[4][0]-ang_min) < 0.3:
                                    if vitima_igual:
                                        self.sign_list.pop(cont)
                                    else:
                                        self.sign_list[cont] = [x_right - x_left, [a, b], [ae, be], [ad, bd], [ang_min, ang_max]]
                                        #print("atualizei", a, b)
                                    vitima_igual = True
                            cont = cont + 1

                        if not vitima_igual and ((abs(ang_max-ang) > 0.2 and abs(ang-ang_min) > 0.2) or dist < 0.8):
                            #print("adicionei 1", a, b)
                            self.sign_list.append([x_right - x_left, [a, b], [ae, be], [ad, bd], [ang_min, ang_max]])
                            cv2.imwrite("vitima_add_" + str(int(a*100)) + "_" + str(int(b*100)) + ".png", img)

    
    '''========================================= AUXILIAR FUNCTIONS ==========================================='''


    def dist_coords(self, a, b):
        dist = ((a[0]-b[0])**2 + (a[1]-b[1])**2)**0.5
        return dist