from controller import Robot
import numpy as np
import cv2
import os
import math

print("começo")

robot = Robot()
time_step = 16
current_tick = 0
calibration_timer = 50

wheel_left = robot.getDevice("wheel1 motor")
wheel_left.setPosition(float("inf"))

wheel_right = robot.getDevice("wheel2 motor")
wheel_right.setPosition(float("inf"))

gps = robot.getDevice("gps")
gps.enable(time_step)
gyro = robot.getDevice("gyro")
gyro.enable(time_step)

camera_left = robot.getDevice("camera1")
camera_left.enable(time_step*5)
camera_right = robot.getDevice("camera2")
camera_right.enable(time_step*5)
camera_left.setFov(1.5)
camera_right.setFov(1.5)
print("cameras", camera_left.getFov(), camera_right.getFov())

def is_wall(bgr_color):
    rgb_color = [bgr_color[2], bgr_color[1], bgr_color[0]]
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

# def get_px(img):
#     for y in range(0, 32): #passa por todas as posições de pixel 
#         for x in range(0, 128):
#             color_px = [img.item(y, x, 2), img.item(y, x, 1), img.item(y, x, 0)]
#             print (color_px) 
#             k = is_wall(color_px)
#             print("pixel ", "X: ", x,"Y: ", y ,"parede? ", k)

# def run_simulation():
#     # update_sensors()

#     global point_cloud

#     if(current_tick % 50 == 0):
        # print("=========================================================")
        # point_cloud = np.array(lidar.getLayerPointCloud(2)[0:512])
        # process_camera(camera_right.getImage(), "right")
        # process_camera(camera_left.getImage(), "left")

# blue---rgb(52, 52, 242)---rgb(14, 14, 52) --- hsv(240, 79, 95) --- hsv(240, 73, 20)
# purple---rgb(120, 52, 201)---rgb(24, 14, 37) --- hsv(267, 74, 79) --- hsv(266, 62, 15)
# red---rgb(242, 52, 52)---rgb(50, 14, 14) --- hsv(0, 88, 58) --- hsv(0, 56, 13)
# green---rgb(27, 236, 27)---rgb(8, 47, 8) --- hsv(120, 89, 93) --- hsv(120, 83, 18)
# orange---rgb(242, 201, 52)---rgb(52, 38, 14) --- hsv(47, 79, 95) --- hsv(38, 73, 20)
# yellow---rgb(242, 242, 52)---rgb(52, 52, 14) --- hsv(60, 79, 95) --- hsv(60, 73, 20)
# swamp---rgb(182, 148, 83)---rgb(35, 29, 20) --- hsv(39, 54, 71) --- hsv(36, 43, 14)
# blackhole---rgb(23, 23, 23)---
# checkpoint---
# floor---rgb(208, 208, 208)---rgb(39, 39, 39) --- hsv(0, 0, 82) --- hsv(0, 0, 15)

# def ground_colour(colour_hsv, colour_bgr):
#     colour_rgb = [colour_bgr[2], colour_bgr[1], colour_bgr[0]]
#     if(colour_hsv[0]>=230 and colour_hsv[0]<=250) and (colour_hsv[1]>=63 and colour_hsv[1]<=84):
#         return 'b'
#     elif(colour_hsv[0]>=267-10 and colour_hsv[0]<=267+10) and (colour_hsv[1]>=74-10 and colour_hsv[1]<=74+10):
#         return 'p'
#     elif(colour_hsv[0]>=0 and colour_hsv[0]<=10) and (colour_hsv[1]>=46 and colour_hsv[1]<=98):
#         return 'r'
#     elif(colour_hsv[0]>=120-10 and colour_hsv[0]<=120+10) and (colour_hsv[1]>=83-25 and colour_hsv[1]<=89+10):
#         return 'g'
#     elif(colour_hsv[0]>=28 and colour_hsv[0]<=57) and (colour_hsv[1]>=63 and colour_hsv[1]<=84):
#         return 'o'
#     elif(colour_hsv[0]>=60-10 and colour_hsv[0]<=60+10) and (colour_hsv[1]>=79-10 and colour_hsv[1]<=79+10):
#         return 'y'
#     elif(colour_hsv[0]>=26 and colour_hsv[0]<=49) and (colour_hsv[1]>=33 and colour_hsv[1]<=64):
#         return 'sw'
#     elif (abs(colour_rgb[0] - 31) <= 5 and abs(colour_rgb[1] - 31) <= 5 and abs(colour_rgb[2] - 31) <= 5) or (abs(colour_rgb[0] - 24) <= 5 and abs(colour_rgb[1] - 24) <= 5 and abs(colour_rgb[2] - 24) <= 5) or (abs(colour_rgb[0] - 11) <= 5 and abs(colour_rgb[1] - 11) <= 5 and abs(colour_rgb[2] - 11) <= 5):
#         return 'bh'
#     elif(colour_hsv[0]>=255 and colour_hsv[0]<=275) and (colour_hsv[1]>=55 and colour_hsv[1]<=80):
#         return '0'
#     else:
#         return 'cp'
def ground_colour(colour_hsv, colour_bgr):
    colour_rgb = [colour_bgr[2], colour_bgr[1], colour_bgr[0]]
    if (abs(colour_rgb[0] - 31) <= 5 and abs(colour_rgb[1] - 31) <= 5 and abs(colour_rgb[2] - 31) <= 5) or (abs(colour_rgb[0] - 24) <= 5 and abs(colour_rgb[1] - 24) <= 5 and abs(colour_rgb[2] - 24) <= 5) or (abs(colour_rgb[0] - 11) <= 5 and abs(colour_rgb[1] - 11) <= 5 and abs(colour_rgb[2] - 11) <= 5):
        return 'bh'
        # buraco
        # elif is_wall(colour_rgb):
        #     return 1
        #     # parede
    elif abs(colour_hsv[0] - 0) <= 3 and abs(colour_hsv[1] - 0) <= 30:
        return '0'
        # chão normal
    elif abs(colour_hsv[0] - 19) <= 3 and abs(colour_hsv[1] - 125) <= 30:
        return 'sw'
        # pantano
    elif abs(colour_hsv[0] - 120) <= 3 and abs(colour_hsv[1] - 190) <= 30:
        return 'b'
        # blue (1-2)
    elif abs(colour_hsv[0] - 133) <= 3 and abs(colour_hsv[1] - 170) <= 30:
        return 'p'
        # purple (2-3)
    elif abs(colour_hsv[0] - 0) <= 3 and abs(colour_hsv[1] - 190) <= 30:
        return 'r'
        # red (3-4)
    elif abs(colour_hsv[0] - 60) <= 3 and abs(colour_hsv[1] - 220) <= 30:
        return 'g'
    elif abs(colour_hsv[0] - 22) <= 3 and abs(colour_hsv[1] - 205) <= 30:
        return 'o'
    elif abs(colour_hsv[0] - 30) <= 3 and abs(colour_hsv[1] - 205) <= 30:
        return 'y'
        #green (1-4)
        # orange = 22, 205, 233
        # yellow = 30, 205, 233
    else:
        return 'cp'
        #checkpoint  


initial_gps = [0, 0]
last_gps = [0, 0]
last_gyro = 0
front_gps = [0, 0]


def update_gps():
    global last_gps
    global front_gps
    last_gps = [gps.getValues()[0] - initial_gps[0], -gps.getValues()[2] + initial_gps[2]]
    front_gps = [last_gps[0] + 0.03284 * math.cos(last_gyro), last_gps[1] + 0.03284 * math.sin(last_gyro)]
    return 

def update_gyro():
    global last_gyro
    ''' 
    Atualiza o Gyro normalizado
    '''
    last_gyro = last_gyro + gyro.getValues()[1]*time_step*0.001
    if last_gyro > math.pi:
        last_gyro -= 2*math.pi
    if last_gyro < -math.pi:
        last_gyro += 2*math.pi
    return

def calibrate_gyro():
    '''
    Sets the initial Gyro values
    '''
    global last_gyro
    last_gyro = math.atan2(last_gps[1] , last_gps[0])
    return

def dfs_border(img, x, y, rgb, visited):
    img[x, y] = 255
    if x < img.shape[0]-1 and img[x+1][y] == rgb and not visited[x+1][y]:
        dfs_border(img, x+1, y, rgb)
    if x > 0 and img[x-1][y] == rgb and not visited[x+1][y]:
        dfs_border(img, x-1, y, rgb)
    if y < img.shape[1]-1 and img[x][y+1] == rgb and not visited[x+1][y]:
        dfs_border(img, x, y+1, rgb)
    if y > 0 and img[x][y-1] == rgb and not visited[x+1][y]:
        dfs_border(img, x, y-1, rgb)

    return img

def bfs_tile(bgr_img, hsv_img, xi, yi, colour):
    menor_x = 129
    maior_x = -1
    menor_y = 41
    maior_y = -1

    #print("inicio bfs", xi, yi, colour)
    
    # queue = [[xi+1, yi], [xi-1, yi], [xi, yi+1], [xi, yi-1]]
    queue = [[xi, yi]]
    #visited = [[[False] * 128] * 40]
    while len(queue) != 0:
        [x, y] = queue[0]
        queue.pop(0)

        #print("queue", x, y)
        #print("if", bgr_img.shape[0]-1, bgr_img.shape[1]-1)

        if y <= bgr_img.shape[0]-1 and y >= 0 and x <= bgr_img.shape[1]-1 and x >= 0:
            colour_bgr = [bgr_img.item(y, x, 0), bgr_img.item(y, x, 1), bgr_img.item(y, x, 2)] 
            colour_hsv = [hsv_img.item(y, x, 0), hsv_img.item(y, x, 1), hsv_img.item(y, x, 2)]

            #print("bfs atual", x, y)
            #print(colour_bgr, colour_hsv, ground_colour(colour_hsv, colour_bgr))
         
            bgr_img[y, x] = [192, 192, 192] # marca como parede p n visitar dnv
            hsv_img[y, x] = [0, 0, 0] # marca como parede p n visitar dnv
            
            if not is_wall(colour_bgr) and ground_colour(colour_hsv, colour_bgr) == colour:
                #print("append")
                queue.append([x+1, y])
                queue.append([x-1, y])
                queue.append([x, y+1])
                queue.append([x, y-1])
            else: continue
                
            menor_x = min(menor_x, x)
            maior_x = max(maior_x, x)
            menor_y = min(menor_y, y)
            maior_y = max(maior_y, y)
    return [bgr_img, hsv_img, [menor_x, maior_x], [menor_y, maior_y]]
    
def run_simulation():
    global current_tick
    global last_gps
    global last_gyro
    global front_gps
    update()

    current_tick += 1
    img_data = camera_left.getImage()
    img = np.array(np.frombuffer(img_data, np.uint8).reshape((camera_left.getHeight(), camera_left.getWidth(), 4)))
    # img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)

    img = cv2.cvtColor(img, cv2.COLOR_BGRA2BGR)
    hsv_img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    # startConnectionTile = -1
    # endConnectionFile = -1
    # if current_tick%50 == 0:
    #     cv2.imshow("a", hsv_img)
    #     cv2.waitKey(0) 
    print("abacate")
    # mydict = {
    #     'key1': 'value1',
    #     'key2': 'value2',
    # }
    # mydict['key1'] == 'value2'
    # list(mydict.keys()) # ['key1', 'key2']
    # connectTilePos = { # 'bloco': [[menorX, maiorX], [menorY, maiorY]], num frame
    #     'b': [[-1, -1], [-1, -1]],
    #     'p': [[-1, -1], [-1, -1]],
    #     'r': [[-1, -1], [-1, -1]],
    #     'g': [[-1, -1], [-1, -1]],
    #     'o': [[-1, -1], [-1, -1]],
    #     'y': [[-1, -1], [-1, -1]],
    #     'sw': [[-1, -1], [-1, -1]], # swamp
    #     'bh': [[-1, -1], [-1, -1]], # black hole
    #     'cp': [[-1, -1], [-1, -1]] # check point
    # }
    connectTilePos = []
    for x in range(0, 128):
        for y in range(39, -1, -1):
            colour_bgr = [img.item(y, x, 0), img.item(y, x, 1), img.item(y, x, 2)] 
            colour_hsv = [hsv_img.item(y, x, 0), hsv_img.item(y, x, 1), hsv_img.item(y, x, 2)] 
            g_colour = ground_colour(colour_hsv, colour_bgr)
            if is_wall(colour_bgr):
                break
            # if current_tick%50 == 0:
            #     print(g_colour, colour_hsv, colour_bgr, y, x)
            if g_colour == '0':
                continue

            [img, hsv_img, [menor_x, maior_x], [menor_y, maior_y]] = bfs_tile(img, hsv_img, x, y, g_colour)
            # print("depois bfs", [menor_x, maior_x], [menor_y, maior_y])
            cv2.imwrite("depoisbfs_bgr.png", img)
            cv2.imwrite("depoisbfs_hsv.png", hsv_img)
            if maior_y != -1: connectTilePos.append([[menor_x, maior_x], [menor_y, maior_y], g_colour])
    # possiveisTiles = list(connectTilePos.keys())
    for i in connectTilePos:
        if i[2] != 'bh': continue

        print("tile diferente", i[2])
        y_up = i[1][0]
        y_down = i[1][1]
        print("ys", y_up, y_down)
        if y_down == -1:
            continue
        # if y_down - y_up < 5: continue
        vertical_fov = 0.566587633
        angle_Y = math.atan((((y_up + y_down)/2)-20) * math.tan(vertical_fov/2) / 20)
        
        # 0.12 - 0.199 (angle_Y)
        # 0.0423 - 0.0303 (cam_height)

        cam_height = (0.0423*(1-((angle_Y-0.12)/(0.199-0.12))) + 0.0303*(((angle_Y-0.12)/(0.199-0.12))))/2
        
        # cam_height = ((0.0423)*(0.12+(0.199-angle_Y)) + (0.0303)*(0.12+angle_Y))/(0.12+0.199)

        cam_height = 0.0303+0.0120
        if angle_Y > 0.199: cam_height -= 0.0120
        #if angle_Y > 
        distance = cam_height/math.tan(angle_Y)
        if distance < 0.12 or distance > 0.3:
            print("dist passou")
            continue 
        print("angle and distance", angle_Y, distance)

        x_left = i[0][0]
        x_right = i[0][1]
        print("xs", x_left, x_right)
        horizontal_fov = 1.5
        angle_X = math.atan((-((x_left + x_right)/2)+64) * math.tan(horizontal_fov/2) / 64)
        print("anglex", angle_X)

        coord_X = front_gps[0] + distance * math.cos(last_gyro + angle_X + 0.75)
        coord_Y = front_gps[1] + distance * math.sin(last_gyro + angle_X + 0.75)
        print(coord_X, coord_Y)

def speed(left_speed, right_speed):
    wheel_left.setVelocity(left_speed)
    wheel_right.setVelocity(right_speed)
    return

def update():
    update_gps()
    update_gyro()
    
def run_calibration():
    global initial_gps
    '''
    Starts the simulation by calibrating the robot 
    '''

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
        update()
    return
        

while robot.step(time_step) != -1:
    current_tick += 1
    if current_tick <= calibration_timer:
        run_calibration()
    else:
        run_simulation()
