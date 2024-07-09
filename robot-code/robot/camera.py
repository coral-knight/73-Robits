import math
import numpy as np
import struct
import cv2


class Camera:

    def __init__(self, hardware, time_step, emitter, gps, gyro, lidar, map):
        self.hardware = hardware
        self.time_step = time_step
        self.emitter = emitter
        self.gps = gps
        self.gyro = gyro
        self.lidar = lidar
        self.map = map

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

        self.c_id = 0
        self.c_turn_velocity = 0
        self.c_initial_tick = True
        self.c_end = False
        self.c_side = "none"

        self.c_found_side = False
        self.c_centered_side = False
        self.c_center_gyro = -5
        self.c_found_center = False
        self.c_centered_center = False
        self.c_identified = False
        self.c_type = 'N'
        self.c_close = False
        self.c_sent = 0
        self.c_timer = 0
        self.c_back = False


    def closest_token(self, coords):
        closest = [-1, 1000]
        for i in range(len(self.sign_list)):
            sign = self.sign_list[i]
            print("sign list", sign)

            if self.dist_coords(coords, sign[1]) < closest[1]:
                ang = math.atan2(sign[1][1] - self.gps.last[1], sign[1][0] - self.gps.last[0])
                [ang_min, ang_max] = sign[4]

                # Check if it's on the right side of the wall
                if ((ang_min >= 0 and (ang > ang_min or ang < ang_max)) or (ang_min < 0 and ang > ang_min and ang < ang_max)) and (abs(ang-ang_min) > 0.47 and 2*math.pi-abs(ang-ang_min) > 0.47) and (abs(ang-ang_max) > 0.47 and 2*math.pi-abs(ang-ang_max) > 0.47):
                    print("right side of wall")
                    closest = [i, self.dist_coords(coords, sign[1])]

        return closest[0]


    def collect(self, navigation, current_tick):
        # Controller for all components and actions of the Collect function

        img, hsv_img = self.joint_image()
        
        if self.c_initial_tick:
            print("=======\nCollect")

            self.c_id = self.closest_token(self.gps.last)
            if self.c_id == -1: self.c_end = True
            self.c_turn_velocity = navigation.turn_velocity-1.5
            self.c_initial_tick = False
            self.c_side = "none"

            self.c_found_side = False
            self.c_centered_side = False
            self.c_center_gyro = -5
            self.c_found_center = False
            self.c_centered_center = False
            self.c_identified = False
            self.c_type = 'N'
            self.c_close = False
            self.c_sent = 0
            self.c_timer = 0
            self.c_back = False 

            self.camera_left.enable(self.time_step)
            self.camera_right.enable(self.time_step)

        if not self.c_end:
            # Align token with a camera
            if not self.c_found_side:
                ang = math.atan2(self.sign_list[self.c_id][1][1]-self.gps.front[1], self.sign_list[self.c_id][1][0]-self.gps.front[0])

                delta_angle_left = ang-(self.gyro.last+0.75+0.25)
                while delta_angle_left < -math.pi: delta_angle_left = delta_angle_left + 2*math.pi
                while delta_angle_left > math.pi: delta_angle_left = delta_angle_left - 2*math.pi

                delta_angle_right = ang-(self.gyro.last-0.75-0.25)
                while delta_angle_right < -math.pi: delta_angle_right = delta_angle_right + 2*math.pi
                while delta_angle_right> math.pi: delta_angle_right = delta_angle_right - 2*math.pi

                if self.c_side == "none":
                    if abs(delta_angle_left) <= abs(delta_angle_right): self.c_side = "left"
                    if abs(delta_angle_left) >= abs(delta_angle_right): self.c_side = "right"

                if self.c_side == "left": delta_angle = delta_angle_left
                if self.c_side == "right": delta_angle = delta_angle_right

                print("turning to token", delta_angle)

                if abs(delta_angle) > 0.035:
                    if delta_angle >= 0: navigation.speed(-self.c_turn_velocity, self.c_turn_velocity)
                    else: navigation.speed(self.c_turn_velocity, -self.c_turn_velocity)
                else:
                    if self.c_side == "left": ray = 450
                    if self.c_side == "right": ray = 60

                    print("dist found", self.lidar.ray_dist(ray, current_tick))
                    if self.lidar.ray_dist(ray, current_tick) > 0.1: 
                        self.c_end = True
                    
                    self.c_found_side = True

            # Center the token within a camera image
            elif not self.c_centered_side:  
                if self.c_side == "left": x = 64
                if self.c_side == "right": x = 192

                left_count, right_count = 0, 0
                while not self.is_wall([hsv_img.item(17, x-left_count, 0), hsv_img.item(17, x-left_count, 1), hsv_img.item(17, x-left_count, 2)]): left_count += 1
                while not self.is_wall([hsv_img.item(17, x+right_count, 0), hsv_img.item(17, x+right_count, 1), hsv_img.item(17, x+right_count, 2)]): right_count += 1
                print("centering", left_count, right_count)

                if left_count == 0 and right_count == 0:
                    if self.c_side == "left": navigation.speed(-self.c_turn_velocity, self.c_turn_velocity)
                    if self.c_side == "right": navigation.speed(self.c_turn_velocity, -self.c_turn_velocity)

                    print("girou", min(abs(self.gyro.last - self.c_center_gyro), 2*math.pi-abs(self.gyro.last - self.c_center_gyro)))

                    if self.c_center_gyro == -5: self.c_center_gyro = self.gyro.last
                    if abs(self.gyro.last - self.c_center_gyro) > 0.7 and 2*math.pi-abs(self.gyro.last - self.c_center_gyro) > 0.7: 
                        self.c_end = True

                if left_count > right_count: navigation.speed(-self.c_turn_velocity, self.c_turn_velocity)
                if right_count > left_count: navigation.speed(self.c_turn_velocity, -self.c_turn_velocity)

                if left_count != 0 and right_count != 0 and abs(left_count - right_count) < 7: 
                    if self.c_side == "left": ray, ang = 450, 0.75
                    if self.c_side == "right": ray, ang = 60, -0.75

                    dist = self.lidar.ray_front_dist(ray, current_tick)
                    token_coords = [self.gps.front[0] + dist * math.cos(self.gyro.last+ang), self.gps.front[1] + dist * math.sin(self.gyro.last+ang)]
                    print("token coords from", self.c_side, token_coords)

                    print("dist center", self.lidar.ray_dist(ray, current_tick))
                    if self.lidar.ray_dist(ray, current_tick) > 0.09: 
                        self.c_end = True
                    else:
                        self.c_id = self.closest_token(token_coords)
                        self.sign_list[self.c_id][1] = token_coords

                    self.c_centered_side = True

            # Identify the token's type
            elif not self.c_identified:  
                navigation.speed(0, 0)
                self.c_type = self.identify_token([img, hsv_img], self.c_side)
                # VERIFICAR SE JA PEGO
                # self.c_type = 'H'

                if self.c_type == 'N': self.c_end = True 

                self.c_identified = True
                print("identified", self.c_type)

            # Align token with the center of both cameras
            elif not self.c_found_center:
                ang = math.atan2(self.sign_list[self.c_id][1][1]-self.gps.last[1], self.sign_list[self.c_id][1][0]-self.gps.last[0])
                delta_angle = ang-self.gyro.last

                print("turning to token center", delta_angle)

                if abs(delta_angle) >= 0.02:
                    while delta_angle < -math.pi: delta_angle = delta_angle + 2*math.pi
                    while delta_angle > math.pi: delta_angle = delta_angle - 2*math.pi

                    if delta_angle >= 0: navigation.speed(-self.c_turn_velocity, self.c_turn_velocity)
                    else: navigation.speed(self.c_turn_velocity, -self.c_turn_velocity)
                else: 
                    self.c_found_center = True

            # Center the token within both cameras
            elif not self.c_centered_center:  
                x = 128

                left_count, right_count = 0, 0
                while not self.is_wall([hsv_img.item(17, x-left_count, 0), hsv_img.item(17, x-left_count, 1), hsv_img.item(17, x-left_count, 2)]): left_count += 1
                while not self.is_wall([hsv_img.item(17, x+right_count, 0), hsv_img.item(17, x+right_count, 1), hsv_img.item(17, x+right_count, 2)]): right_count += 1
                print("centering", left_count, right_count)

                if left_count > right_count: navigation.speed(-self.c_turn_velocity, self.c_turn_velocity)
                if right_count > left_count: navigation.speed(self.c_turn_velocity, -self.c_turn_velocity)

                if abs(left_count - right_count) < 6: 
                    navigation.speed(0, 0)
                    self.c_centered_center = True

            # Get closer to send the right token
            elif not self.c_close:  
                print("get closer")
                navigation.speed(navigation.turn_velocity, navigation.turn_velocity)
                if self.lidar.ray_dist(0, current_tick) < 0.05:
                    print("done")
                    self.c_close = True

            # Send the token with emitter
            elif self.c_sent <= 0: 
                navigation.speed(0, 0)
                self.c_timer += 1
                print(self.c_timer*self.time_step)

                if self.c_sent != -1 and self.c_timer*self.time_step > 1500:
                    dist = self.lidar.ray_front_dist(0, current_tick)
                    sign_coords = [self.gps.front[0] + dist * math.cos(self.gyro.last), self.gps.front[1] + dist * math.sin(self.gyro.last)]
                    sign_type = bytes(self.c_type, "utf-8")
                    print("time to send")
                    print("coords", sign_coords)

                    print("message coords", [int((sign_coords[0]+self.gps.initial[0])*100), int((-sign_coords[1]+self.gps.initial[2])*100)])

                    message = struct.pack("i i c", int((sign_coords[0]+self.gps.initial[0])*100), int((-sign_coords[1]+self.gps.initial[2])*100), sign_type)
                    self.emitter.send(message)

                    self.map.add_extra(sign_coords, self.c_type)

                    self.c_id = self.closest_token(sign_coords)
                    self.c_sent = -1
                
                if self.c_timer*self.time_step > 1550: 
                    print("finished sending")
                    self.c_sent = 1

            # Get back to initial position
            elif not self.c_back:  
                print("get back")
                navigation.speed(-navigation.turn_velocity, -navigation.turn_velocity)
                if self.lidar.ray_dist(0, current_tick) > 0.055:
                    self.c_back = True
                    self.c_end = True
                    print("done")

        if self.c_end:
            print("finished collecting")

            self.c_initial_tick = True
            self.c_end = False
            if self.c_id != -1:
                print("deleted", self.sign_list[self.c_id][1])
                self.sign_colleted.append(self.sign_list[self.c_id])
                self.sign_list.pop(self.c_id)
            self.camera_left.enable(self.time_step*5)
            self.camera_right.enable(self.time_step*5)

            return False

        return True
    

    def find_square_vertices(self, v1, v2):
        # Calculate the midpoint
        mid_x = (v1[0] + v2[0]) / 2
        mid_y = (v1[1] + v2[1]) / 2

        # Calculate the vector from v1 to v2
        vec_x = v2[0] - v1[0]
        vec_y = v2[1] - v1[1]

        # Perpendicular vector (rotated by 90 degrees)
        perp_vec_x = -vec_y
        perp_vec_y = vec_x

        # Calculate half of the length of the side of the square
        half_side_length = np.sqrt(perp_vec_x**2 + perp_vec_y**2) / 2

        # Normalize the perpendicular vector
        norm = np.sqrt(perp_vec_x**2 + perp_vec_y**2)
        perp_vec_x /= norm
        perp_vec_y /= norm

        # Calculate the other two vertices
        v3 = (mid_x + perp_vec_x * half_side_length, mid_y + perp_vec_y * half_side_length)
        v4 = (mid_x - perp_vec_x * half_side_length, mid_y - perp_vec_y * half_side_length)

        return v3, v4

 
    def find_vertex(self, hsv_img):
        mid = [64, 20] # [x, y]
        visited = []
        queue = []
        visited.append(mid)
        queue.append(mid)

        vertices = []
        x_least_top = mid[0]
        x_greater_top = mid[0]
        x_least_bot = mid[0]
        x_greater_bot = mid[0]
        left_up = mid
        left_down = mid
        right_up = mid
        right_down = mid
        up_left = mid
        up_right = mid
        down_left = mid 
        down_right = mid

        while queue:
            current = queue.pop(0)

            x_current = current[0]
            y_current = current[1]

            if y_current == 0:
                x_least_top = min(x_current, x_least_top)
                x_greater_top = max(x_current, x_greater_top)
            if y_current == 39:
                x_least_bot = min(x_current, x_least_bot)
                x_greater_bot = max(x_current, x_greater_bot)

            # Caso mais esquerda, em cima
            if x_current < left_up[0] or (x_current == left_up[0] and y_current <= left_up[1]):
                left_up = [x_current, y_current]
            # Caso mais a esquerda, embaixo
            if x_current < left_down[0] or (x_current == left_down[0] and y_current >= left_down[1]):
                left_down = [x_current, y_current]
            # Caso direita, cima
            if x_current > right_up[0] or (x_current == right_up[0] and y_current <= right_up[1]):
                right_up = [x_current, y_current]
            # Caso direita, baixo 
            if x_current > right_down[0] or (x_current == right_down[0] and y_current >= right_down[1]):
                right_down = [x_current, y_current]

            # Caso cima, esquerda
            if y_current < up_left[1] or (y_current == up_left[1] and x_current <= up_left[0]):
                up_left = [x_current, y_current]
            # Caso cima, direita 
            if y_current < up_right[1] or (y_current == up_right[1] and x_current >= up_right[0]):
                up_right = [x_current, y_current]
            # Caso baixo, esquerda
            if y_current > down_left[1] or (y_current == down_left[1] and x_current <= down_left[0]):
                down_left = [x_current, y_current]
            # Caso baixo, direita 
            if y_current > down_right[1] or (y_current == down_right[1] and x_current >= down_right[0]):
                down_right = [x_current, y_current]

            # Checa se e vertice
            cont_adj = 0

            if(y_current < 39):
                px = [hsv_img.item(y_current+1, x_current, 0), hsv_img.item(y_current+1, x_current, 1), hsv_img.item(y_current+1, x_current, 2)]
                if(not self.is_wall(px)): cont_adj += 1
            if(y_current > 0):
                px = [hsv_img.item(y_current-1, x_current, 0), hsv_img.item(y_current-1, x_current, 1), hsv_img.item(y_current-1, x_current, 2)]
                if(not self.is_wall(px)): cont_adj += 1
            if(x_current < 127):
                px = [hsv_img.item(y_current, x_current+1, 0), hsv_img.item(y_current, x_current+1, 1), hsv_img.item(y_current, x_current+1, 2)]
                if(not self.is_wall(px)): cont_adj += 1
            if(x_current > 0):
                px = [hsv_img.item(y_current, x_current-1, 0), hsv_img.item(y_current, x_current-1, 1), hsv_img.item(y_current, x_current-1, 2)]
                if(not self.is_wall(px)): cont_adj += 1

            if(cont_adj == 1 or cont_adj == 2):
                angle_center = math.atan2(y_current-mid[1], x_current-mid[0])
                vertices.append([angle_center, [x_current, y_current]])
                
            for y in range(-1, 2, 1):
                for x in range(-1, 2, 1):
                    x_next = x_current+x
                    y_next = y_current+y
                    if(x_next >= 0 and y_next >= 0 and x_next < 128 and y_next < 40):
                        px = [hsv_img.item(y_next, x_next, 0), hsv_img.item(y_next, x_next, 1), hsv_img.item(y_next, x_next, 2)]
                        if not self.is_wall(px) and [x_next, y_next] not in visited:
                            visited.append([x_next, y_next])
                            queue.append([x_next, y_next])

        if len(vertices) == 0: return [[0, 0], [0, 0], [0, 0], [0, 0]]

        cont_last_column = 0
        for xi in range (left_up[0], right_up[0]+1, 1):
            cont_cur_column = 0
            for yi in range (up_left[1], down_left[1]+1, 1):
                px = [hsv_img.item(yi, xi, 0), hsv_img.item(yi, xi, 1), hsv_img.item(yi, xi, 2)]
                if not self.is_wall(px):
                    cont_cur_column += 1

            if cont_last_column != 0 and abs(cont_cur_column - cont_last_column) >= 10:
                vertices.sort()
                sorted_vertices = []
                for i in vertices: sorted_vertices.append(i[1])

                return self.visvalingam_whyatt(sorted_vertices)
            
            cont_last_column = cont_cur_column
    
        if x_greater_top - x_least_top >= 1 or x_greater_bot - x_least_bot >= 1:
            v3, v4 = self.find_square_vertices(left_down, right_up)
            angleToCenter = math.atan2(v3[1]-mid[1], v3[0]-mid[0])
            vertices.append([angleToCenter, [v3[0], v3[1]]])

            angleToCenter = math.atan2(v4[1]-mid[1], v4[0]-mid[0])
            vertices.append([angleToCenter, [v4[0], v4[1]]])

        vertices.sort()
        sorted_vertices = []
        for i in vertices: sorted_vertices.append(i[1])

        return self.visvalingam_whyatt(sorted_vertices)


    def identify_letter(self, gray_img):
        middle_black = False
        for y in range(-10, 11, 1):
            for x in range(-10, 11, 1):
                if gray_img.item(125+y, 125+x) <= 1: middle_black = True

        if not middle_black: return 'U'
        else:
            # Count white-black on the middle line
            counter_white_black = 0
            pre_black = False

            for x in range(249):
                black = False
                for y in range(115, 136, 1): 
                    if gray_img.item(y, x) <= 1: black = True
                if black == True and pre_black == False: counter_white_black += 1
                pre_black = black

            if counter_white_black >= 3: return 'S'

            # Count white-black on the middle column
            counter_white_black = 0
            pre_black = False

            for y in range(249):
                black = False
                for x in range(115, 136, 1): 
                    if gray_img.item(y, x) <= 1: black = True
                if black == True and pre_black == False: counter_white_black += 1
                pre_black = black

            if counter_white_black >= 3: return 'S'
                
            return 'H'


    def identify_token(self, joint, side):
        [img, hsv_img] = joint
        print("side of identify", side)
        if side == "left": 
            print("left")
            img = img[0:40, 0:128]
            hsv_img = hsv_img[0:40, 0:128]
        if side == "right": 
            print("right")
            img = img[0:40, 128:256]
            hsv_img = hsv_img[0:40, 128:256]

        cv2.imwrite("identify.png", img)

        vertexes = self.find_vertex(hsv_img)
        vertexes[0][1] += 40
        vertexes[1][1] += 40
        vertexes[2][1] += 40
        vertexes[3][1] += 40
        if vertexes[0] == vertexes[1] and vertexes[1] == vertexes[2] and vertexes[2] == vertexes[3]: return 'N'

        img = cv2.copyMakeBorder(img, 40, 40, 0, 0, cv2.BORDER_CONSTANT, None, value = [255, 255, 255])
        hsv_img = cv2.cvtColor(np.float32(img), cv2.COLOR_BGR2HSV)

        cv2.imwrite("border_rgb.png", img)
        cv2.imwrite("border_hsv.png", hsv_img)

        # Warping
        resolution = 250
        pts1 = np.float32(vertexes)
        pts2 = np.float32([[0, 0], [resolution, 0], [resolution, resolution], [0, resolution]])
        M = cv2.getPerspectiveTransform(pts1, pts2)
        warped = cv2.warpPerspective(np.float32(img), M, (resolution, resolution))        
        warped_hsv = cv2.cvtColor(warped, cv2.COLOR_BGR2HSV)

        cv2.imwrite("warped.png", warped)

        # Define the ranges of the colors
        lower_red_1 = np.array([0, 0.5, 0])
        upper_red_1 = np.array([30, 1.0, 360])
        lower_red_2 = np.array([330, 0.5, 0])
        upper_red_2 = np.array([360, 1.0, 360])

        lower_yellow = np.array([10, 0.5, 0])
        upper_yellow = np.array([80, 1.0, 360])
        
        # Mask 
        mask_red = cv2.inRange(warped_hsv, lower_red_1, upper_red_1)
        mask_red += cv2.inRange(warped_hsv, lower_red_2, upper_red_2)
        mask_yellow = cv2.inRange(warped_hsv, lower_yellow, upper_yellow)

        #count the number of pixels from each mask 
        red = np.sum(mask_red == 255)
        yellow = np.sum(mask_yellow == 255)

        print("red", red)
        print("yellow", yellow)

        #verify the masks to check Flamables and Organics
        if yellow > 100: return 'O'
        elif red > 100: return 'F'
        
        warped_gray = cv2.cvtColor(warped, cv2.COLOR_BGR2GRAY)
        
        # Binary value for corrosive
        _, warped_binary = cv2.threshold(warped_gray, 200, 255, cv2.THRESH_BINARY)
        percentage = np.sum(warped_binary == 0)/62500.0
        print("corrosive per", percentage)
        if(percentage > 0.75): return 'C'
        
        # Binary value for poison
        _, warped_binary = cv2.threshold(warped_gray, 100, 255, cv2.THRESH_BINARY)
        percentage = np.sum(warped_binary == 0)/62500.0
        print("poison per", percentage)
        if(percentage < 0.20): return 'P'
        
        cv2.imwrite("warped_binary.png", warped_binary)
        return self.identify_letter(warped_binary)


    def seen(self, current_tick):
        ray_left = round((math.pi-(1.2))*256/math.pi + 255.5)
        ray_left = ray_left % 512

        ray_right = round((math.pi+(1.2))*256/math.pi + 255.5)
        ray_right = ray_right % 512

        ray = ray_left
        while ray != ray_right:
            [coordX, coordY] = self.lidar.ray_coords(ray, 2, current_tick)

            dist = self.dist_coords(self.gps.front, [coordX, coordY])
            if dist < 0.24:
                d = int(dist/0.02) if int(dist/0.02) > 0 else 1
                for i in range(d+1): 
                    [x, y] = [((d-i)*self.gps.front[0]+i*coordX)/d, ((d-i)*self.gps.front[1]+i*coordY)/d]
                    if self.dist_coords([x, y],  self.map.closest([x, y], 1)) > 0.023:
                        self.map.seen([x, y])
                    else:
                        continue

            ray += 1
            if ray == 512: ray = 0


    def find(self, image):
        img = image

        topwall = np.zeros(256, dtype=int)
        deltas = []
        x_right = -1 
        x_left = 257

        for y in range(1,39): 
            for x in range(0, 256): 
                colour_hsv1 = [img.item(y, x, 0), img.item(y, x, 1), img.item(y, x, 2)] 
                colour_hsv2 = [img.item(y+1, x, 0), img.item(y+1, x, 1), img.item(y+1, x, 2)]
                colour_hsv3 = [img.item(y-1, x, 0), img.item(y-1, x, 1), img.item(y-1, x, 2)]

                if self.is_wall(colour_hsv1) and self.is_wall(colour_hsv2) and not self.is_wall(colour_hsv3): 
                    if topwall[x] == 1: 
                        topwall[x] = y 

                        up_camera = colour_hsv3
                        right_camera = colour_hsv3

                        up_count = 0
                        right_count = 0

                        while((y-up_count) > 0 and not self.is_wall(up_camera)):
                            up_count = up_count + 1
                            up_camera = [img.item(y-up_count, x, 0), img.item(y-up_count, x, 1), img.item(y-up_count, x, 2)]

                        up_count = int((up_count+1)/2)

                        while((x+right_count) < 255 and not self.is_wall(right_camera)):
                            right_count = right_count + 1
                            right_camera = [img.item(y-up_count, x+right_count, 0), img.item(y-up_count, x+right_count, 1), img.item(y-up_count, x+right_count, 2)]

                        if right_count >= 2:
                            new = True
                            for d in deltas: 
                                if (x < d[1] + 1 and x+right_count > d[0] - 1): new = False
                            
                            if new:
                                x_left = x
                                x_right = x+right_count-1
                                deltas.append([x_left, x_right])

                    else: 
                        ceiling = [img.item(0, x, 0), img.item(0, x, 1), img.item(0, x, 2)] 

                        if self.is_wall(ceiling):
                            up_camera = colour_hsv3 
                            right_camera = colour_hsv3  

                            up_count = 0 
                            right_count = 0  

                            while((y-up_count) > 0 and not self.is_wall(up_camera)): 
                                up_count = up_count + 1 
                                up_camera = [img.item(y-up_count, x, 0), img.item(y-up_count, x, 1), img.item(y-up_count, x, 2)] 

                            up_count = int((up_count+1)/2) 

                            while((x+right_count) < 255 and not self.is_wall(right_camera)):
                                right_count = right_count + 1
                                right_camera = [img.item(y-up_count, x+right_count, 0), img.item(y-up_count, x+right_count, 1), img.item(y-up_count, x+right_count, 2)] 

                            if right_count >= 2:
                                new = True
                                for d in deltas: 
                                    if (x < d[1] + 1 and x+right_count > d[0] - 1): new = False
                                
                                if new:
                                    x_left = x
                                    x_right = x+right_count-1
                                    deltas.append([x_left, x_right])

                        elif topwall[x] == 0: 
                            topwall[x] = 1

        return deltas, topwall


    def update_token(self, current_tick): 
        #print("process cameras =========")

        img, hsv_img = self.joint_image()
        deltas, topwall = self.find(hsv_img)

        for d in deltas:
            [x_left, x_right] = d
            
            if abs(topwall[x_right]-topwall[x_left]) <= 6 and (topwall[x_right] > 1 or self.is_wall([hsv_img.item(0, x_right, 0), hsv_img.item(0, x_right, 1), hsv_img.item(0, x_right, 2)])):
                #print("TOKEN ===", (x_left+x_right)/2, "============================")
                #print("aqui", x_left, x_right)
                #cv2.imwrite("possible_victim_" + str(tick_count) + ".png", img)

                if ((x_left+x_right)/2) <= 128: 
                    img_angle = math.atan((-((x_left+x_right)/2)+63.5) * math.tan(1.5/2) / 63.5) + 0.75
                if ((x_left+x_right)/2) > 128:
                    img_angle = math.atan((-((x_left+x_right)/2-128)+63.5) * math.tan(1.5/2) / 63.5) - 0.75
                
                raio = round( ((math.pi-(img_angle))*256/math.pi) + 255.5 )
                raio = raio % 512

                dist = self.lidar.ray_front_dist(raio, current_tick)
                #print("raio", raio)
                #print("dist", dist)
                #print("img angle", img_angle)
                #print("ang total", self.gyro.last+img_angle)

                if dist < 0.8:
                    a = self.gps.front[0] + dist * (math.cos(self.gyro.last+img_angle))
                    b = self.gps.front[1] + dist * (math.sin(self.gyro.last+img_angle))

                    #print("coords", a, b)

                    rd = 1
                    if dist < 0.08: rd = 3

                    aux = 2*math.pi/512

                    ae = self.gps.front[0] + self.lidar.ray_front_dist((raio-rd+512)%512, current_tick) * (math.cos(self.gyro.last+img_angle+rd*aux))
                    be = self.gps.front[1] + self.lidar.ray_front_dist((raio-rd+512)%512, current_tick) * (math.sin(self.gyro.last+img_angle+rd*aux))
                    ad = self.gps.front[0] + self.lidar.ray_front_dist((raio+rd)%512, current_tick) * (math.cos(self.gyro.last+img_angle-rd*aux))
                    bd = self.gps.front[1] + self.lidar.ray_front_dist((raio+rd)%512, current_tick) * (math.sin(self.gyro.last+img_angle-rd*aux))

                    ang = math.atan2(b - self.gps.last[1], a - self.gps.last[0]) 
                    ang_max = math.atan2(be-bd, ae-ad)
                    ang_min = math.atan2(bd-be, ad-ae)
                    
                    rd = 3
                    ae = self.gps.front[0] + self.lidar.ray_front_dist((raio-rd+512)%512, current_tick) * (math.cos(self.gyro.last+img_angle+rd*aux))
                    be = self.gps.front[1] + self.lidar.ray_front_dist((raio-rd+512)%512, current_tick) * (math.sin(self.gyro.last+img_angle+rd*aux))
                    ad = self.gps.front[0] + self.lidar.ray_front_dist((raio+rd)%512, current_tick) * (math.cos(self.gyro.last+img_angle-rd*aux))
                    bd = self.gps.front[1] + self.lidar.ray_front_dist((raio+rd)%512, current_tick) * (math.sin(self.gyro.last+img_angle-rd*aux))

                    #print("coords left", ae, be)
                    #print("coords right", ad, bd)
                    #print("dist left", self.dist_coords([ae, be], [a, b]))
                    #print("dist right", self.dist_coords([a, b], [ad, bd]))
                    #print("dist left-right", self.dist_coords([ae, be], [ad, bd]))

                    if (self.dist_coords([ae, be], [ad, bd]) < 0.06) and (self.dist_coords([ae, be], [a, b]) < 0.03) and (self.dist_coords([a, b], [ad, bd]) < 0.03):
                        vitima_igual = False

                        for v in self.sign_colleted:
                            if (self.dist_coords([a, b], v[1]) < 0.033 and (abs(v[4][0]-ang_min) < math.pi/3 or 2*math.pi-abs(v[4][0]-ang_min) < math.pi/3)):
                                vitima_igual = True

                        c = 0
                        for v in self.sign_list:
                            if self.dist_coords([a, b], v[1]) < 0.033:
                                if abs(v[4][0]-ang_min) < 0.6 or 2*math.pi-abs(v[4][0]-ang_min) < 0.6:
                                    if vitima_igual:
                                        self.sign_list.pop(c)
                                    elif x_right - x_left >= v[0]:
                                        self.sign_list[c] = [x_right - x_left, [a, b], [ae, be], [ad, bd], [ang_min, ang_max]]
                                    vitima_igual = True
                            c = c + 1

                        
                        if not vitima_igual and ((abs(ang-ang_min) > 0.2 and 2*math.pi-abs(ang-ang_min) > 0.2) and (abs(ang-ang_max) > 0.2 and 2*math.pi-abs(ang-ang_max) > 0.2) or dist < 0.24):
                            print("ADDED TOKEN", a, b)
                            self.sign_list.append([x_right - x_left, [a, b], [ae, be], [ad, bd], [ang_min, ang_max]])
                            cv2.imwrite("vitima_add_" + str(int(a*100)) + "_" + str(int(b*100)) + ".png", img)


    def print_list(self):
        for s in self.sign_list:
            print("sign:")
            print(s)


    def pixel_ground_position(self, pos):
        [x, y] = pos
        #if x == 1000 or y <= 19: return [1000, 1000]

        horizontal_fov = 1.5
        vertical_fov = 0.566587633

        tan_angle_Y = (y-19.5) * math.tan(vertical_fov/2) / 19.5
        d_min = 0.03/tan_angle_Y

        #if d_min > 0.28: return [1000, 1000]

        if x < 128: 
            angle_X = math.atan((-x+63.5) * math.tan(horizontal_fov/2) / 63.5)
            distance = d_min / math.cos(angle_X)
            angle_X += 0.75

        if x >= 128:
            angle_X = math.atan((-(x-128)+63.5) * math.tan(horizontal_fov/2) / 63.5)
            distance = d_min / math.cos(angle_X)
            angle_X -= 0.75

        coord_X = self.gps.front[0] + distance * math.cos(self.gyro.last + angle_X)
        coord_Y = self.gps.front[1] + distance * math.sin(self.gyro.last + angle_X)

        return [coord_X, coord_Y]


    def bfs_tile(self, hsv_img, xi, yi):
        img = hsv_img.copy()
        initial_hsv = [hsv_img.item(yi, xi, 0), hsv_img.item(yi, xi, 1), hsv_img.item(yi, xi, 2)]
        colour = self.identify_colour(initial_hsv)

        values, diff_value = [hsv_img.item(yi, xi, 2)], 1
        min_top = 40
        ground = []
        tiles = []

        area = 0
        vis = np.zeros([256,40])
        queue = [[xi, yi]]
        while len(queue) != 0 and area < 10000:
            [x, y] = queue[0]
            queue.pop(0)

            if y <= 39 and y >= 0 and x <= 255 and x >= 0:
                if vis[x, y]: continue
                vis[x, y] = 1
                area += 1

                colour_hsv = [hsv_img.item(y, x, 0), hsv_img.item(y, x, 1), hsv_img.item(y, x, 2)]

                '''if y > 0: top_hsv = [hsv_img.item(y-1, x, 0), hsv_img.item(y-1, x, 1), hsv_img.item(y-1, x, 2)]
                if y < 39: bottom_hsv = [hsv_img.item(y+1, x, 0), hsv_img.item(y+1, x, 1), hsv_img.item(y+1, x, 2)]
                if x < 255: right_hsv = [hsv_img.item(y, x+1, 0), hsv_img.item(y, x+1, 1), hsv_img.item(y, x+1, 2)]
                if x > 0: left_hsv = [hsv_img.item(y, x-1, 0), hsv_img.item(y, x-1, 1), hsv_img.item(y, x-1, 2)]'''
                
                if self.is_blank_ground([img.item(y, x, 0), img.item(y, x, 1), img.item(y, x, 2)]): continue
                if abs(initial_hsv[0] - colour_hsv[0]) > 10 or abs(initial_hsv[1] - colour_hsv[1]) > 0.1: continue

                if all(abs(colour_hsv[2] - v) > 10 for v in values):
                    values.append(colour_hsv[2])
                    diff_value += 1

                min_top = min(min_top, y)

                '''top, bot, left, right = False, False, False, False
                if y > 0 and (not self.is_wall(top_hsv) and (abs(initial_hsv[0] - top_hsv[0]) > 10 or abs(initial_hsv[1] - top_hsv[1]) > 0.1 or (initial_hsv[0] < 10 and initial_hsv[1] < 0.1 and top_hsv[2] > 188))):
                    top = True
                if y < 39 and (not self.is_wall(bottom_hsv) and (abs(initial_hsv[0] - bottom_hsv[0]) > 10 or abs(initial_hsv[1] - bottom_hsv[1]) > 0.1 or (initial_hsv[0] < 10 and initial_hsv[1] < 0.1 and bottom_hsv[2] > 188))):
                    bot = True
                if x > 0 and (not self.is_wall(left_hsv) and (abs(initial_hsv[0] - left_hsv[0]) > 10 or abs(initial_hsv[1] - left_hsv[1]) > 0.1 or (initial_hsv[0] < 10 and initial_hsv[1] < 0.1 and left_hsv[2] > 188))):
                    left = True
                if x < 255 and (not self.is_wall(right_hsv) and (abs(initial_hsv[0] - right_hsv[0]) > 10 or abs(initial_hsv[1] - right_hsv[1]) > 0.1 or (initial_hsv[0] < 10 and initial_hsv[1] < 0.1 and right_hsv[2] > 188))):
                    right = True
                        
                if bot and left and right: ground.append([x, y-2])
                elif bot and left: ground.append([x+2, y-1])
                elif bot and right: ground.append([x-2, y-1])
                elif bot: ground.append([x, y-2])'''

                coords = self.pixel_ground_position([x, y])
                dist = self.dist_coords(self.gps.last, coords)

                if coords != [1000, 1000] and dist < 0.25:
                    a, b = int(coords[0] / 0.06), int(coords[1] / 0.06)
                    if a != 0: a = int((a+(a/abs(a)))/2)
                    if b != 0: b = int((b+(b/abs(b)))/2)

                    minx, miny = a*0.12-0.06, b*0.12-0.06
                    maxx, maxy = a*0.12+0.06, b*0.12+0.06

                    if dist < 0.15: border = 0.03
                    else: border = 0.045

                    if abs(coords[0]-minx) > border and abs(coords[0]-maxx) > border and abs(coords[1]-miny) > border and abs(coords[1]-maxy) > border:
                        if all([a*0.12, b*0.12] != x for x in tiles): 
                            if colour != 'ob': 
                                cv2.imwrite("added" + str(a) + "_" + str(b) + "_" + colour + ".png", hsv_img)
                                #print(a*0.12, b*0.12)
                                #print(x, y)
                                #print("coords", coords)
                                #print("dist", dist)
                            tiles.append([a*0.12, b*0.12])

                queue.append([x+1, y])
                queue.append([x-1, y])
                queue.append([x, y+1])
                queue.append([x, y-1])

                img[y, x] = [0, 0, 192] # paint as normal ground

        if area == 10000: print("while quebrou")

        return [tiles, min_top, diff_value, ground, img]


    def is_obstacle(self, top, diff_value, ground, colour):
        obstacle = False

        #print("top", top)
        #print("diff value", diff_value)

        if top < 18: obstacle = True # Top beyond the middle of image
        if diff_value > 3 and colour != 'cp': obstacle = True # Change on the brightness value of HSV

        '''if obstacle:
            ground.sort()
            for i in range(min(5, len(ground))): ground.pop(0)
            for i in range(min(5, len(ground))): ground.pop(len(ground)-1)

            for [i, j] in ground:
                if not all(([a, b] == [i, j] or abs(b - j) > 3) for [a, b] in ground):
                    [coord_X, coord_Y] = self.pixel_ground_position([i, j-1])
                    self.map.add_obstacle([coord_X, coord_Y])
                    self.map.add_extra([coord_X, coord_Y], 'ob')
                    print("added obstacle", [i, j], [coord_X, coord_Y])'''

        return obstacle
        

    def update_ground(self):
        img, hsv_img = self.joint_image()

        for x in range(0, 256):
            if self.is_wall([hsv_img.item(0, x, 0), hsv_img.item(0, x, 1), hsv_img.item(0, x, 2)]): continue 
            for y in range(39, 0, -1):
                colour_hsv = [hsv_img.item(y, x, 0), hsv_img.item(y, x, 1), hsv_img.item(y, x, 2)] 
                g_colour = self.identify_colour(colour_hsv)

                if self.is_wall(colour_hsv): break
                if self.is_blank_ground(colour_hsv): continue

                cv2.imwrite("current.png", img)

                #print("TEM COISA", x, y, "----------------------------")
                #print("colour", g_colour)
                #print("pos", self.gps.last)

                [tiles, top, diff_value, ground, hsv_img] = self.bfs_tile(hsv_img, x, y)

                if self.is_obstacle(top, diff_value, ground, g_colour): 
                    #print("OBSTACLE")
                    #cv2.imwrite("obstacle_" + str(x) + "_" + str(y) + ".png", hsv_img) 
                    continue

                if g_colour == 'N': continue

                #print("tiles", tiles)

                if g_colour == 'bh': a = 5
                else: a = 3

                for t in tiles:
                    for i in range(a):
                        for j in range(a):
                            minx, maxx = t[0]-(a/100), t[0]+(a/100)
                            miny, maxy = t[1]-(a/100), t[1]+(a/100)
                            coord = [(((a-1)-i)*minx+i*maxx)/(a-1), (((a-1)-j)*miny+j*maxy)/(a-1)]
                            self.map.add_extra(coord, g_colour)
                            if g_colour == 'bh': self.map.add_obstacle(coord)
                
        return

    
    '''========================================= AUXILIAR FUNCTIONS ==========================================='''


    def dist_coords(self, a, b):
        dist = ((a[0]-b[0])**2 + (a[1]-b[1])**2)**0.5
        return dist
    
    
    def joint_image(self):
        # Get cameras' images and join them into a single one

        img_data_l = self.camera_left.getImage()
        img_data_r = self.camera_right.getImage()
        img_l = np.array(np.frombuffer(img_data_l, np.uint8).reshape((self.camera_left.getHeight(), self.camera_left.getWidth(), 4)))
        img_r = np.array(np.frombuffer(img_data_r, np.uint8).reshape((self.camera_right.getHeight(), self.camera_right.getWidth(), 4)))

        img = np.zeros([40,256,4], dtype="int")
        img[0:, 0:128] = img_l[0:, 0:]
        img[0:, 128:256] = img_r[0:, 0:]

        hsv_img = cv2.cvtColor(np.float32(img), cv2.COLOR_BGR2HSV)

        return img, hsv_img
    

    def identify_colour(self, colour_hsv):
        if abs(colour_hsv[0] - 240) <= 10 and abs(colour_hsv[1] - 0.75) <= 0.1: return 'b' # blue (1-2)
        if abs(colour_hsv[0] - 60) <= 10 and abs(colour_hsv[1] - 0.75) <= 0.1: return 'y' # yellow (1-3)
        if abs(colour_hsv[0] - 120) <= 10 and abs(colour_hsv[1] - 0.85) <= 0.1: return 'g' # green (1-4)
        if abs(colour_hsv[0] - 266) <= 10 and abs(colour_hsv[1] - 0.70) <= 0.1: return 'p' # purple (2-3)
        if abs(colour_hsv[0] - 44) <= 10 and abs(colour_hsv[1] - 0.75) <= 0.1: return 'o' # orange (2-4)
        if abs(colour_hsv[0] - 0) <= 10 and abs(colour_hsv[1] - 0.75) <= 0.1: return 'r' # red (3-4)

        if abs(colour_hsv[0] - 225) <= 10 and colour_hsv[1] <= 0.50: return 'cp' # checkpoint  
        if abs(colour_hsv[0] - 38) <= 10 and abs(colour_hsv[1] - 0.55) <= 0.1: return 'sw' # swamp
        if abs(colour_hsv[0] - 189) <= 10 and abs(colour_hsv[1] - 0.50) <= 0.1: return 'wa' # wall
        if abs(colour_hsv[0] - 0) <= 10 and abs(colour_hsv[1] - 0) <= 0.1 and colour_hsv[2] < 32: return 'bh' # black hole
        if abs(colour_hsv[0] - 0) <= 10 and abs(colour_hsv[1] - 0) <= 0.1 and colour_hsv[2] > 188: return '0' # normal ground
        if abs(colour_hsv[0] - 0) <= 10 and abs(colour_hsv[1] - 0) <= 0.1: return 'ob'

        return 'N'


    def is_blank_ground(self, colour_hsv):
        if abs(colour_hsv[0] - 0) <= 10 and abs(colour_hsv[1] - 0) <= 0.1 and colour_hsv[2] > 188: return True
        return False


    def is_wall(self, colour_hsv):
        #colour_hsv = cv2.cvtColor(np.float32([[colour_rgb]]), cv2.COLOR_RGB2HSV)[0][0]
        if abs(colour_hsv[0] - 189) <= 10 and abs(colour_hsv[1] - 0.50) <= 0.1: return True
        return False


    def triangle_area(self, p1, p2, p3): 
        # Triangle area using coordinates p1, p2 and p3 with matrix determinant
        return abs((p1[0]*(p2[1] - p3[1]) + p2[0]*(p3[1] - p1[1]) + p3[0]*(p1[1] - p2[1])) / 2.0)


    def visvalingam_whyatt(self, coords):
        n = len(coords)
        areas = [] # area[i][0] = Self area, area[i][1] = [x, y], vertice coordinate

        for i in range(0, len(coords)):
            area = self.triangle_area(coords[(i-1)%n], coords[i], coords[(i+1)%n])
            areas.append([area, coords[i]])
        
        while len(areas) > 4:
            n = len(areas)
            # Get minimal area
            minIndex = 0
            for i in range(0, len(areas)):
                if areas[i][0] < areas[minIndex][0]: minIndex = i
            
            # Update adjacents areas
            areas[(minIndex-1)%n][0] = self.triangle_area(areas[(minIndex-2)%n][1], areas[(minIndex-1)%n][1], areas[(minIndex+1)%n][1])
            areas[(minIndex+1)%n][0] = self.triangle_area(areas[(minIndex-1)%n][1], areas[(minIndex+1)%n][1], areas[(minIndex+2)%n][1])

            del areas[minIndex%n]

        print("areas", len(areas))
        return [areas[0][1], areas[1][1], areas[2][1], areas[3][1]]


    def centroid(self, vertexes):
        vertexes = np.array(vertexes)
        length = vertexes.shape[0]
        sum_x = np.sum(vertexes[:, 0])
        sum_y = np.sum(vertexes[:, 1])
        return [sum_x/length, sum_y/length]