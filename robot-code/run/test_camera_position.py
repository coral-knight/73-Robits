from controller import Robot
import cv2
from collections import deque
import numpy as np
import os


robot = Robot()
time_step = 16

receiver = robot.getDevice("receiver")
receiver.enable(time_step)

wheel_left = robot.getDevice("wheel1 motor")
wheel_right = robot.getDevice("wheel2 motor")


#camera_front = robot.getDevice("camera1")
camera_right = robot.getDevice("camera1")
camera_right.enable(time_step*5)
camera_left = robot.getDevice("camera2")
camera_left.enable(time_step*5)

camera_left.setFov(1.5)
camera_right.setFov(1.5)

def find_victim(img):
    topwall = np.zeros(128, dtype=int)
    deltas = []
    x_right = -1
    x_left = 65

    for y in range(2, 28):
        for x in range(0, 128):
            colour_rgb1 = [img.item(y, x, 2), img.item(y, x, 1), img.item(y, x, 0)]
            colour_rgb2 = [img.item(y+1, x, 2), img.item(y+1, x, 1), img.item(y+1, x, 0)]
            colour_rgb3 = [img.item(y-1, x, 2), img.item(y-1, x, 1), img.item(y-1, x, 0)]

            if is_wall(colour_rgb1) and is_wall(colour_rgb2) and not is_wall(colour_rgb3):
                if topwall[x] == 1:
                    topwall[x] = y

                    up_camera = colour_rgb3
                    left_camera = colour_rgb3

                    up_count = 0
                    left_count = 0

                    # enquanto não for parede, up_camera
                    while((y-up_count) > 0 and not is_wall(up_camera)):
                        up_count = up_count + 1
                        up_camera = [img.item(y-up_count, x, 2), img.item(y-up_count, x, 1), img.item(y-up_count, x, 0)]

                    up_count = int((up_count+1)/2)

                    # enquanto não for parede, left_camera
                    while((x+left_count) < 63 and not is_wall(left_camera)):
                        left_count = left_count + 1
                        left_camera = [img.item(y-up_count, x+left_count, 2), img.item(y-up_count, x+left_count, 1), img.item(y-up_count, x+left_count, 0)]

                    if x > x_right + 1 or x+left_count-1 < x_left - 1:
                        x_left = x
                        x_right = x+left_count-1
                        if camera_print:
                            print("vítima", x_left, x_right, y-up_count, len(deltas)) 

                        deltas.append([x_left, x_right])
                else:
                    up_camera = [img.item(0, x, 2), img.item(0, x, 1), img.item(0, x, 0)]
                    if is_wall(up_camera):
                        #if camera_print:
                            #print("aqui tem coisa sem dois", x, y)

                        up_camera = colour_rgb3
                        left_camera = colour_rgb3

                        up_count = 0
                        left_count = 0

                        # enquanto for parede, up_camera
                        while((y-up_count) > 0 and not is_wall(up_camera)):
                            up_count = up_count + 1
                            up_camera = [img.item(y-up_count, x, 2), img.item(y-up_count, x, 1), img.item(y-up_count, x, 0)]

                        up_count = int((up_count+1)/2)

                        # enquanto for parede, left_camera
                        while((x+left_count) < 63 and not is_wall(left_camera)):
                            left_count = left_count + 1
                            left_camera = [img.item(y-up_count, x+left_count, 2), img.item(y-up_count, x+left_count, 1), img.item(y-up_count, x+left_count, 0)]


                        if x > x_right + 1 or x+left_count-1 < x_left - 1:
                            x_left = x
                            x_right = x+left_count-1
                            if camera_print:
                                print("vítima2", x_left, x_right, len(deltas)) 
                            
                            deltas.append([x_left, x_right])
                    elif topwall[x] == 0:
                        topwall[x] = 1
    return deltas, topwall


def process_camera(img_data, type_camera):
    global point_cloud

    img = np.array(np.frombuffer(img_data, np.uint8).reshape((camera_front.getHeight(), camera_front.getWidth(), 4)))
    #("img" + "_" + type_camera + ".png", img)

    deltas, topwall = find_victim(img)

    for d in deltas:
        [x_left, x_right] = d
        #if camera_print:
            #print(d, topwall[x_right], topwall[x_left], is_wall([img.item(0, x_right, 2), img.item(0, x_right, 1), img.item(0, x_right, 0)]))
        if x_right - x_left >= 1 and abs(topwall[x_right]-topwall[x_left]) < 5 and (topwall[x_right] > 1 or is_wall([img.item(0, x_right, 2), img.item(0, x_right, 1), img.item(0, x_right, 0)])):
            if camera_print:
                print("aqui", x_left, x_right, type_camera)
            #cv2.imwrite("possible_victim_" + str(tick_count) + ".png", img)

            if type_camera == "front":
                img_angle = math.atan((-((x_left+x_right)/2)+31) * math.tan(0.84/2) / 32)
            if type_camera == "plus":
                img_angle = math.atan((-((x_left+x_right)/2)+31) * math.tan(0.84/2) / 32) + 1.03
            if type_camera == "sub":
                img_angle = math.atan((-((x_left+x_right)/2)+31) * math.tan(0.84/2) / 32) - 1.03
            img_angle = img_angle + (31 - (x_left+x_right)/2) / 31 * 0.12
            raio = round((math.pi-(img_angle))*256/math.pi)
            raio = (raio+255) % 512

            dist = dist_lidar_2(raio)
            if camera_print:
                print("dist", dist)

            if dist < 0.18:
                a = front_gps[0] + dist * (math.sin(total_gyro+img_angle))
                b = front_gps[1] + dist * (math.cos(total_gyro+img_angle))

                rd = 1

                if dist < 0.08:
                    rd = 3

                aux = 2*math.pi/512

                ae = front_gps[0] + dist_lidar_2((raio-rd+512)%512) * (math.sin(total_gyro+img_angle+rd*aux))
                be = front_gps[1] + dist_lidar_2((raio-rd+512)%512) * (math.cos(total_gyro+img_angle+rd*aux))
                ad = front_gps[0] + dist_lidar_2((raio+rd)%512) * (math.sin(total_gyro+img_angle-rd*aux))
                bd = front_gps[1] + dist_lidar_2((raio+rd)%512) * (math.cos(total_gyro+img_angle-rd*aux))

                ang = math.atan2(a - last_gps[0], b - last_gps[1]) 
                ang_max = math.atan2(ae-ad, be-bd)
                ang_min = math.atan2(ad-ae, bd-be)
                
                ae = front_gps[0] + dist_lidar_2((raio-6+512)%512) * (math.sin(total_gyro+img_angle+6*aux))
                be = front_gps[1] + dist_lidar_2((raio-6+512)%512) * (math.cos(total_gyro+img_angle+6*aux))
                ad = front_gps[0] + dist_lidar_2((raio+6)%512) * (math.sin(total_gyro+img_angle-6*aux))
                bd = front_gps[1] + dist_lidar_2((raio+6)%512) * (math.cos(total_gyro+img_angle-6*aux))

                if (dist_coords(ae, be, ad, bd) < 0.05) and (dist_coords(ae, be, a, b) < 0.03) and (dist_coords(a, b, ad, bd) < 0.03):
                    walla, wallb = map.coords_to_tile(a, b)

                    if camera_print:
                        print(ang_min)
                        print("achei essa", walla, wallb)
                        print(dist_coords(ae, be, ad, bd))

                    cont = 0
                    vitima_igual = False

                    if camera_print:
                        print("vai ver done")
                    for v in victim_done:
                        wav, wbv = map.coords_to_tile(v[1], v[2])
                        if camera_print:
                            print(wav, wbv, v[3])
                        if (dist_coords(a, b, v[1], v[2]) < 0.033 and (abs(v[3]-ang_min) < math.pi/3 or 2*math.pi-abs(v[3]-ang_min) < math.pi/3)):
                            if camera_print:
                                print("já pegou")
                            vitima_igual = True

                    if camera_print:
                        print("certo", len(victim_list))
                    for v in victim_list:
                        wav, wbv = map.coords_to_tile(v[1], v[2])
                        if dist_coords(a, b, v[1], v[2]) < 0.033:
                            if camera_print:
                                print("vitima perto")
                                print(wav, wbv, v[3])
                            if abs(v[3]-ang_min) < 0.3 or 2*math.pi-abs(v[3]-ang_min) < 0.3:
                                if vitima_igual:
                                    victim_list.pop(cont)
                                else:
                                    victim_list[cont] = [v[0], a, b, ang_min, ang_max, x_right - x_left, [ae, be], [ad, bd]]
                                    if camera_print:
                                        print("atualizei", walla, wallb)
                                vitima_igual = True
                        cont = cont + 1

                    if not vitima_igual and ((abs(ang_max-ang) > 0.2 and abs(ang-ang_min) > 0.2) or dist < 0.8):
                        print("adicionei 1", walla, wallb)
                        victim_list.append([1, a, b, ang_min, ang_max, x_right - x_left, [ae, be], [ad, bd]])
                        #cv2.imwrite("vitima_crop_" + str(walla) + "_" +str(wallb) + "_" + type_camera + ".png", img)

def get_image():

    #image_path = r'C:\Users\OBR\Desktop\EREBUS_image\camera_in\img1.png'
    print("bom dia")
    print(camera_left.getFov())
    print(camera_left.getMaxFov())
    # Image directory 
    #directory = r'C:\Users\OBR\Desktop\EREBUS_image\camera_in_128x32'
    
    #img_cam1 = camera_front.getImage()
    img_cam2 = camera_right.getImage()
    img_cam3 = camera_left.getImage()  
    #img_front = np.array(np.frombuffer(img_cam1, np.uint8).reshape((camera_front.getHeight(), camera_front.getWidth(), 4)))
    img_right = np.array(np.frombuffer(img_cam2, np.uint8).reshape((camera_right.getHeight(), camera_right.getWidth(), 4)))
    img_left = np.array(np.frombuffer(img_cam3, np.uint8).reshape((camera_left.getHeight(), camera_left.getWidth(), 4)))

    #os.chdir(directory) 

    #cv2.imwrite(filename, img_front) 
    #cv2.imwrite(filename, img_right) 
    #cv2.imwrite(filename, img_left) 

    #print("After saving image:")   
    #print(os.listdir(directory)) 
    
    #print('Successfully saved') 

cont_img = 0
tick_count = 0
while robot.step(time_step) != -1:
    tick_count+=1
    if(tick_count%80):
        cont_img+=1
        get_image()