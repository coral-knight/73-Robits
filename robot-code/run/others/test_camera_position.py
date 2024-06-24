from controller import Robot
import numpy as np
import cv2
import math

print("começo")

robot = Robot()
time_step = 16
current_tick = 0
calibration_timer = 50

wheel_left = robot.getDevice("wheel1 motor")
wheel_right = robot.getDevice("wheel2 motor")

print("rodas")

camera_left = robot.getDevice("camera1")
camera_left.enable(time_step*5)
camera_right = robot.getDevice("camera2")
camera_right.enable(time_step*5)
camera_left.setFov(1.5)
camera_right.setFov(1.5)
print("cameras", camera_left.getFov(), camera_right.getFov())

victim_list = []

gps = robot.getDevice("gps")
gps.enable(time_step)
initial_gps = [0, 0]
last_gps = [0, 0]
front_gps = [0, 0]

print("gps")

lidar = robot.getDevice("lidar")
lidar.enable(time_step*5)
lidar.enablePointCloud()
point_cloud = []

print("lidar")

gyro = robot.getDevice("gyro")
gyro.enable(time_step)
last_gyro = 0 

print("gyro")

print("depois sensores")


def update_sensors():
    update_gyro()
    update_gps()

def update_gps():
        global last_gps
        global front_gps

        last_gps = [gps.getValues()[0] - initial_gps[0], -gps.getValues()[2] + initial_gps[2]]
        front_gps = [last_gps[0] + 0.03284 * math.cos(last_gyro), last_gps[1] + 0.03284 * math.sin(last_gyro)]
        return 

def update_gyro():
        global last_gyro
        last_gyro = last_gyro + gyro.getValues()[1]*time_step*0.001
        if last_gyro > math.pi:
            last_gyro -= 2*math.pi
        if last_gyro < -math.pi:
            last_gyro += 2*math.pi
        return

def calibrate_gyro():
    global last_gyro
    last_gyro = math.atan2(last_gps[1] , last_gps[0])
    return

def speed(left_speed, right_speed):        
    wheel_left.setVelocity(left_speed)
    wheel_right.setVelocity(right_speed)
    return


def is_wall(rgb_color):
    hsv_color = cv2.cvtColor(np.array([[[rgb_color[2], rgb_color[1], rgb_color[0]]]], dtype=np.uint8), cv2.COLOR_BGR2HSV)[0][0]
    #if rgb_color == [18,30,32]: print(hsv_color)
    yes = False

    if abs(hsv_color[0] - 94) <= 3 and abs(hsv_color[1] - 112) <= 45:
        yes = True

    #if abs(rgb_color[1]-2*rgb_color[0]) < 10 and abs(rgb_color[2]-rgb_color[1]-(rgb_color[1]-rgb_color[0])/6) < 3:
    #    yes = True

    if (rgb_color[0] == rgb_color[1] and rgb_color[1] == rgb_color[2]):
        yes = False
    if (rgb_color[2]-rgb_color[1] > abs(rgb_color[1]-rgb_color[0])):
        yes = False

    return yes

def dist_coords(a, b):
    dist = ((a[0]-b[0])**2 + (a[1]-b[1])**2)**0.5
    return dist

def dist_lidar_2(raio):
    dist = ((point_cloud[raio].x)**2+(point_cloud[raio].y)**2)**0.5
    return dist
     

def find_victim(img):
    topwall = np.zeros(128, dtype=int)
    deltas = []
    x_right = -1
    x_left = 65

    for y in range(2, 28):
        for x in range(0, 128):
            color_rgb1 = [img.item(y, x, 2), img.item(y, x, 1), img.item(y, x, 0)]
            color_rgb2 = [img.item(y+1, x, 2), img.item(y+1, x, 1), img.item(y+1, x, 0)]
            color_rgb3 = [img.item(y-1, x, 2), img.item(y-1, x, 1), img.item(y-1, x, 0)]

            if is_wall(color_rgb1) and is_wall(color_rgb2) and not is_wall(color_rgb3):
                if topwall[x] == 1:
                    topwall[x] = y

                    up_camera = color_rgb3
                    left_camera = color_rgb3

                    up_count = 0
                    left_count = 0

                    # enquanto não for parede, up_camera
                    while((y-up_count) > 0 and not is_wall(up_camera)):
                        up_count = up_count + 1
                        up_camera = [img.item(y-up_count, x, 2), img.item(y-up_count, x, 1), img.item(y-up_count, x, 0)]

                    up_count = int((up_count+1)/2)

                    # enquanto não for parede, left_camera
                    while((x+left_count) < 127 and not is_wall(left_camera)):
                        left_count = left_count + 1
                        left_camera = [img.item(y-up_count, x+left_count, 2), img.item(y-up_count, x+left_count, 1), img.item(y-up_count, x+left_count, 0)]
                        print("next left", left_camera)

                    if x > x_right + 1 or x+left_count-1 < x_left - 1:
                        x_left = x
                        x_right = x+left_count-1
                        print("vítima", x_left, x_right, y-up_count, up_count*2, len(deltas)) 

                        deltas.append([x_left, x_right])
                else:
                    up_camera = [img.item(0, x, 2), img.item(0, x, 1), img.item(0, x, 0)]
                    if is_wall(up_camera):
                        #if camera_print:
                            #print("aqui tem coisa sem dois", x, y)

                        up_camera = color_rgb3
                        left_camera = color_rgb3

                        up_count = 0
                        left_count = 0

                        # enquanto for parede, up_camera
                        while((y-up_count) > 0 and not is_wall(up_camera)):
                            up_count = up_count + 1
                            up_camera = [img.item(y-up_count, x, 2), img.item(y-up_count, x, 1), img.item(y-up_count, x, 0)]

                        up_count = int((up_count+1)/2)

                        # enquanto for parede, left_camera
                        while((x+left_count) < 127 and not is_wall(left_camera)):
                            left_count = left_count + 1
                            left_camera = [img.item(y-up_count, x+left_count, 2), img.item(y-up_count, x+left_count, 1), img.item(y-up_count, x+left_count, 0)]


                        if x > x_right + 1 or x+left_count-1 < x_left - 1:
                            x_left = x
                            x_right = x+left_count-1
                            print("vítima2", x_left, x_right, y-up_count, up_count*2, len(deltas)) 
                            
                            deltas.append([x_left, x_right])
                    elif topwall[x] == 0:
                        topwall[x] = 1
    return deltas, topwall

def process_camera(img_data, type_camera):
    global point_cloud
    global victim_list

    print("camera", type_camera)

    img = np.array(np.frombuffer(img_data, np.uint8).reshape((camera_right.getHeight(), camera_right.getWidth(), 4)))
    cv2.imwrite("img" + "_" + type_camera + ".png", img)

    deltas, topwall = find_victim(img)

    for d in deltas:
        [x_left, x_right] = d
        #if camera_print:
            #print(d, topwall[x_right], topwall[x_left], is_wall([img.item(0, x_right, 2), img.item(0, x_right, 1), img.item(0, x_right, 0)]))
        if x_right - x_left >= 1 and abs(topwall[x_right]-topwall[x_left]) < 5 and (topwall[x_right] > 1 or is_wall([img.item(0, x_right, 2), img.item(0, x_right, 1), img.item(0, x_right, 0)])):
            print("aqui", x_left, x_right)
            #cv2.imwrite("possible_victim_" + str(tick_count) + ".png", img)

            if type_camera == "left":
                img_angle = math.atan((-((x_left+x_right)/2)+63) * math.tan(1.5/2) / 64) + 1.03
            if type_camera == "right":
                img_angle = math.atan((-((x_left+x_right)/2)+63) * math.tan(1.5/2) / 64) - 1.03
            img_angle = img_angle + (63 - (x_left+x_right)/2) / 63 * 0.12
            raio = round((math.pi-(img_angle))*256/math.pi)
            raio = (raio+255) % 512
            print("raio", raio)
            dist = dist_lidar_2(raio)
            print("dist", dist)

            if dist < 0.18:
                a = front_gps[0] + dist * (math.cos(last_gyro+img_angle))
                b = front_gps[1] + dist * (math.sin(last_gyro+img_angle))

                rd = 1

                if dist < 0.08:
                    rd = 3

                aux = 2*math.pi/512

                ae = front_gps[0] + dist_lidar_2((raio-rd+512)%512) * (math.sin(last_gyro+img_angle+rd*aux))
                be = front_gps[1] + dist_lidar_2((raio-rd+512)%512) * (math.cos(last_gyro+img_angle+rd*aux))
                ad = front_gps[0] + dist_lidar_2((raio+rd)%512) * (math.sin(last_gyro+img_angle-rd*aux))
                bd = front_gps[1] + dist_lidar_2((raio+rd)%512) * (math.cos(last_gyro+img_angle-rd*aux))

                ang = math.atan2(a - last_gps[0], b - last_gps[1]) 
                ang_max = math.atan2(ae-ad, be-bd)
                ang_min = math.atan2(ad-ae, bd-be)
                
                ae = front_gps[0] + dist_lidar_2((raio-6+512)%512) * (math.sin(last_gyro+img_angle+6*aux))
                be = front_gps[1] + dist_lidar_2((raio-6+512)%512) * (math.cos(last_gyro+img_angle+6*aux))
                ad = front_gps[0] + dist_lidar_2((raio+6)%512) * (math.sin(last_gyro+img_angle-6*aux))
                bd = front_gps[1] + dist_lidar_2((raio+6)%512) * (math.cos(last_gyro+img_angle-6*aux))

                if (dist_coords([ae, be], [ad, bd]) < 0.05) and (dist_coords([ae, be], [a, b]) < 0.03) and (dist_coords([a, b], [ad, bd]) < 0.03):
                    walla, wallb = map.coords_to_tile(a, b)

                    print(ang_min)
                    print("achei essa", walla, wallb)
                    print(dist_coords([ae, be], [ad, bd]))

                    cont = 0
                    vitima_igual = False

                    print("certo", len(victim_list))
                    for v in victim_list:
                        wav, wbv = map.coords_to_tile(v[1], v[2])
                        if dist_coords([a, b], [v[1], v[2]]) < 0.033:
                            print("vitima perto")
                            print(wav, wbv, v[3])
                            if abs(v[3]-ang_min) < 0.3 or 2*math.pi-abs(v[3]-ang_min) < 0.3:
                                if vitima_igual:
                                    victim_list.pop(cont)
                                else:
                                    victim_list[cont] = [v[0], a, b, ang_min, ang_max, x_right - x_left, [ae, be], [ad, bd]]
                                    print("atualizei", walla, wallb)
                                vitima_igual = True
                        cont = cont + 1

                    if not vitima_igual and ((abs(ang_max-ang) > 0.2 and abs(ang-ang_min) > 0.2) or dist < 0.8):
                        print("adicionei 1", walla, wallb)
                        victim_list.append([1, a, b, ang_min, ang_max, x_right - x_left, [ae, be], [ad, bd]])
                        #cv2.imwrite("vitima_crop_" + str(walla) + "_" +str(wallb) + "_" + type_camera + ".png", img)


def run_calibration():
    global initial_gps

    if current_tick == 1:
        initial_gps = gps.getValues()

    update_gps()

    if current_tick < 5:
        speed(2, 2)
    elif current_tick == 5:
        calibrate_gyro()
    elif current_tick < 10:
        speed(-2, -2)
    else:
        speed(0, 0)
        update_sensors()
    return

def run_simulation():
    update_sensors()

    global point_cloud

    if(current_tick % 50 == 0):
        print("=========================================================")
        point_cloud = np.array(lidar.getLayerPointCloud(2)[0:512])
        process_camera(camera_right.getImage(), "right")
        process_camera(camera_left.getImage(), "left")


while robot.step(time_step) != -1:
    '''current_tick += 1
    if current_tick <= calibration_timer:
        run_calibration()
    else:
        run_simulation()'''
    
    a1 = camera_right.getImage()
    a2 = camera_left.getImage()
    b1 = np.array(np.frombuffer(a2, np.uint8).reshape((camera_right.getHeight(), camera_right.getWidth(), 4)))
    b2 = np.array(np.frombuffer(a1, np.uint8).reshape((camera_right.getHeight(), camera_right.getWidth(), 4)))


    b = np.empty([32,256,4])

    b[0:, 0:128] = b1[0:, 0:]
    b[0:, 128:256] = b2[0:, 0:]

    cv2.imwrite("joint.png", b)
    
