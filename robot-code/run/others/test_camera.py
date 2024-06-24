from controller import Robot
import math
import struct
import cv2
import queue
from collections import deque
import numpy as np
import os

#pega robô
robot = Robot()

# Pega o objeto "roda da esquerda"
wheel_left = robot.getDevice("wheel2 motor")
wheel_left.setPosition(float("inf"))
# Pega o objeto "roda da direita"
wheel_right = robot.getDevice("wheel1 motor")
wheel_right.setPosition(float("inf"))

#time 
time_step = 16

# Pega o giroscópio
gyro = robot.getDevice("gyro")
gyro.enable(time_step)
total_gyro = 0

# Pega o GPS
gps = robot.getDevice("gps")
gps.enable(time_step)
initial_gps = [gps.getValues()[0], gps.getValues()[2]]
last_gps = [0, 0]

#pega a câmera 
camera_right = robot.getDevice("camera2")
camera_right.enable(time_step*5) #Nao
camera_left = robot.getDevice("camera1")
camera_left.enable(time_step*5)
camera_print = False 

camera_right.setFov(1.5)
camera_left.setFov(1.5)

# Pega o LiDAR
lidar = robot.getDevice("lidar")
lidar.enable(time_step*5)
lidar.enablePointCloud()



def dist_coords(x1, y1, x2, y2):
        dist = ((x1-x2)**2 + (y1-y2)**2)**0.5
        return dist

def dist_lidar(raio):
    point = point_cloud[raio]

    angle = total_gyro + math.pi/2

    coordX = front_gps[0] + math.cos(angle) * (point.x) - math.sin(angle) * (point.y)
    coordY = front_gps[1] + math.sin(angle) * (-point.x) - math.cos(angle) * (point.y)

    dist = dist_coords(last_gps[0], last_gps[1], coordX, coordY)
    return dist

def dist_lidar_2(raio):
    dist = ((point_cloud[raio].x)**2+(point_cloud[raio].y)**2)**0.5
    return dist

'''def is_wall(color_px): # color [2, 1, 0] BGR 
    yes = False # yes = 0
    if color_px[0]==0:
        g_r = 0
    # if color_px[1]==0:
    #     g_r = 0
    if color_px[2]==0:
        rg_b = 0
    if color_px[0]!=0 and color_px[1]!=0 and color_px[2]!=0:
        g_r = math.atan2(color_px[0], color_px[1]) 
        rg_b = math.atan2(color_px[2], math.sqrt(color_px[0]^2 + color_px[1]^2))
    if g_r > 0.440 and g_r < 0.545 and rg_b > 1.340 and rg_b < 1.530:
        yes = True
    # print (g_r, rg_b)
    return yes    '''


def is_wall(color):
    yes = False

    if abs(color[1]-2*color[0]) < 10 and abs(color[2]-color[1]-(color[1]-color[0])/6) < 3:
        yes = True

    if (color[0] == color[1] and color[1] == color[2]):
        yes = False
    if (color[2]-color[1] > abs(color[1]-color[0])):
        yes = False

    return yes


def find_sign(img):
    topwall = np.zeros(128, dtype=int) 
    deltas = [] 
    x_right = -1 
    x_left = 129 

    for y in range(1, 31): 
        for x in range(0, 128): 
            colour_rgb1 = [img.item(y, x, 2), img.item(y, x, 1), img.item(y, x, 0)] 
            colour_rgb2 = [img.item(y+1, x, 2), img.item(y+1, x, 1), img.item(y+1, x, 0)]
            colour_rgb3 = [img.item(y-1, x, 2), img.item(y-1, x, 1), img.item(y-1, x, 0)]

            if is_wall(colour_rgb1) and is_wall(colour_rgb2) and not is_wall(colour_rgb3): 
                if topwall[x] == 1: 
                    topwall[x] = y 

                    up_camera = colour_rgb3
                    right_camera = colour_rgb3

                    up_count = 0
                    right_count = 0

                    # enquanto não for parede, up_camera
                    while((y-up_count) > 0 and not is_wall(up_camera)):
                        up_count = up_count + 1
                        up_camera = [img.item(y-up_count, x, 2), img.item(y-up_count, x, 1), img.item(y-up_count, x, 0)]
                        #print("going up", (y-up_count), x, up_camera)

                    up_count = int((up_count+1)/2)

                    # enquanto não for parede, right_camera
                    while((x+right_count) < 128 and not is_wall(right_camera)):
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
                    if is_wall(ceiling):

                        # canto inferior esquerdo da vítima
                        up_camera = colour_rgb3 
                        right_camera = colour_rgb3  

                        up_count = 0 
                        right_count = 0  

                        # enquanto for parede, up_camera
                        while((y-up_count) > 0 and not is_wall(up_camera)): 
                            up_count = up_count + 1 
                            up_camera = [img.item(y-up_count, x, 2), img.item(y-up_count, x, 1), img.item(y-up_count, x, 0)] 
                            #print("going up 2", (y-up_count), x, up_camera)

                        up_count = int((up_count+1)/2) 

                        # enquanto for parede, right_camera
                        while((x+right_count) < 127 and not is_wall(right_camera)):
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



def process_camera(img_data, type_camera):
    global point_cloud

    img = np.array(np.frombuffer(img_data, np.uint8).reshape((camera_right.getHeight(), camera_right.getWidth(), 4)))
    #("img" + "_" + type_camera + ".png", img)

    print("=========")
    deltas, topwall = find_sign(img)

    for d in deltas:
        [x_left, x_right] = d
        
        if abs(topwall[x_right]-topwall[x_left]) < 5 and (topwall[x_right] > 1 or is_wall([img.item(0, x_right, 2), img.item(0, x_right, 1), img.item(0, x_right, 0)])):
            print("aqui", x_left, x_right, type_camera)
            #cv2.imwrite("possible_victim_" + str(tick_count) + ".png", img)

            if type_camera == "left":
                img_angle = math.atan((-((x_left+x_right)/2)+64) * math.tan(1.5/2) / 64) + 0.75
            if type_camera == "right":
                img_angle = math.atan((-((x_left+x_right)/2)+64) * math.tan(1.5/2) / 64) - 0.75

            #img_angle = img_angle + (63 - (x_left+x_right)/2) / 63 * 0.12
            raio = round((math.pi-(img_angle))*256/math.pi)
            raio = (raio+255) % 512

            dist = dist_lidar_2(raio)
            print("dist", dist)
            print("raio", raio)
            print("img angle", img_angle)
            print("ang total", total_gyro+img_angle)

            if dist < 0.18:
                a = front_gps[0] + dist * (math.cos(total_gyro+img_angle))
                b = front_gps[1] + dist * (math.sin(total_gyro+img_angle))

                print("coords", a, b)

                rd = 1

                if dist < 0.08:
                    rd = 3

                aux = 2*math.pi/512

                ae = front_gps[0] + dist_lidar_2((raio-rd+512)%512) * (math.cos(total_gyro+img_angle+rd*aux))
                be = front_gps[1] + dist_lidar_2((raio-rd+512)%512) * (math.sin(total_gyro+img_angle+rd*aux))
                ad = front_gps[0] + dist_lidar_2((raio+rd)%512) * (math.cos(total_gyro+img_angle-rd*aux))
                bd = front_gps[1] + dist_lidar_2((raio+rd)%512) * (math.sin(total_gyro+img_angle-rd*aux))

                ang = math.atan2(b - last_gps[1], a - last_gps[0]) 
                ang_max = math.atan2(be-bd, ae-ad)
                ang_min = math.atan2(bd-be, ad-ae)
                
                ae = front_gps[0] + dist_lidar_2((raio-6+512)%512) * (math.cos(total_gyro+img_angle+6*aux))
                be = front_gps[1] + dist_lidar_2((raio-6+512)%512) * (math.sin(total_gyro+img_angle+6*aux))
                ad = front_gps[0] + dist_lidar_2((raio+6)%512) * (math.cos(total_gyro+img_angle-6*aux))
                bd = front_gps[1] + dist_lidar_2((raio+6)%512) * (math.sin(total_gyro+img_angle-6*aux))

                if (dist_coords(ae, be, ad, bd) < 0.05) and (dist_coords(ae, be, a, b) < 0.03) and (dist_coords(a, b, ad, bd) < 0.03):
                    #walla, wallb = map.coords_to_tile(a, b)

                    print("wall angles", ang, ang_min, ang_max)
                    #print("achei essa", walla, wallb)
                    print(dist_coords(ae, be, ad, bd))

                    cont = 0
                    vitima_igual = False

                    print("vai ver done")
                    for v in victim_done:
                        #wav, wbv = map.coords_to_tile(v[1], v[2])
                        #print(wav, wbv, v[3])
                        if (dist_coords(a, b, v[1], v[2]) < 0.033 and (abs(v[3]-ang_min) < math.pi/3 or 2*math.pi-abs(v[3]-ang_min) < math.pi/3)):
                            print("já pegou")
                            vitima_igual = True

                    print("certo", len(victim_list))
                    for v in victim_list:
                        #wav, wbv = map.coords_to_tile(v[1], v[2])
                        if dist_coords(a, b, v[1], v[2]) < 0.033:
                            print("vitima perto")
                            #print(wav, wbv, v[3])
                            if abs(v[3]-ang_min) < 0.3 or 2*math.pi-abs(v[3]-ang_min) < 0.3:
                                if vitima_igual:
                                    victim_list.pop(cont)
                                else:
                                    victim_list[cont] = [v[0], a, b, ang_min, ang_max, x_right - x_left, [ae, be], [ad, bd]]
                                    print("atualizei", a, b)
                                vitima_igual = True
                        cont = cont + 1

                    if not vitima_igual and ((abs(ang_max-ang) > 0.2 and abs(ang-ang_min) > 0.2) or dist < 0.8):
                        print("adicionei 1", a, b)
                        victim_list.append([1, a, b, ang_min, ang_max, x_right - x_left, [ae, be], [ad, bd]])
                        #cv2.imwrite("vitima_crop_" + str(walla) + "_" +str(wallb) + "_" + type_camera + ".png", img)'''

tick_count = 0
initial_gps = [0, 0]
last_gps = [0, 0]
front_gps = [0, 0]
total_gyro = 0
speed = [0, 0]
point_cloud = np.zeros(512)
victim_list = []
victim_done = []


def run_calibration():
    '''
    Starts the simulation by calibrating the robot 
    '''
    global point_cloud
    global speed
    global total_gyro
    global initial_gps

    if tick_count == 1:
        initial_gps = gps.getValues()

    last_gps = [gps.getValues()[0] - initial_gps[0], - gps.getValues()[2] + initial_gps[2]]
    point_cloud = np.array(lidar.getLayerPointCloud(2)[0:512])

    if tick_count < 5:
        speed = [2, 2]
    elif tick_count == 5:
        total_gyro = math.atan2(last_gps[1] , last_gps[0])
        print("total gyro inicial", total_gyro)
    elif tick_count < 10:
        speed = [-2, -2]
    else:
        speed = [0, 0]
    return


def run_simulation():
    global last_gps
    global point_cloud
    global front_gps 
    global total_gyro

    last_gps = [gps.getValues()[0] - initial_gps[0], - gps.getValues()[2] + initial_gps[2]]
    front_gps[0] = last_gps[0] + 0.03284 * math.cos(total_gyro)
    front_gps[1] = last_gps[1] + 0.03284 * math.sin(total_gyro)

    total_gyro = total_gyro + gyro.getValues()[1]*time_step*0.001
    if total_gyro > math.pi:
        total_gyro -= 2*math.pi
    if total_gyro < -math.pi:
        total_gyro += 2*math.pi

    point_cloud = np.array(lidar.getLayerPointCloud(2)[0:512])

    if tick_count % 80 == 0:
        process_camera(camera_left.getImage(), "left")
        process_camera(camera_right.getImage(), "right")


while robot.step(time_step) != -1:
    tick_count += 1
    if tick_count <= 20:
        run_calibration()
    else:
        run_simulation()
    
    wheel_left.setVelocity(speed[0])
    wheel_right.setVelocity(speed[1])


            
                        


