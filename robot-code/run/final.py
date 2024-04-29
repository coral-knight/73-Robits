# ====== =====    ====   ====  ====  = ===== =====
#     =      =    =   = =    = =   = =   =   =
#    =     ===    ====  =    = ====  =   =   =====
#   =        =    =  =  =    = =   = =   =       =
#  =     =====    =   =  ====  ====  =   =   =====


# -------------------------------------------------------------------------------


# Importação de bibliotecas
from controller import Robot
import math
import struct
import cv2
import queue
from collections import deque
import numpy as np


# -------------------------------------------------------------------------------


# Variáveis padrão

# Define a velocidade de execução do código
time_step = 16

# Cria o robô
robot = Robot()

# Pega o objeto "roda da esquerda"
wheel_left = robot.getDevice("wheel1 motor")
wheel_left.setPosition(float("inf"))
# Pega o objeto "roda da direita"
wheel_right = robot.getDevice("wheel2 motor")
wheel_right.setPosition(float("inf"))

# Pega o GPS
gps = robot.getDevice("gps")
gps.enable(time_step)
initial_gps = [gps.getValues()[0], gps.getValues()[2]]
lop_gps = [0, 0]
last_gps = [0, 0]
front_gps = [0, 0]

# Pega o giroscópio
gyro = robot.getDevice("gyro")
gyro.enable(time_step)
last_gyro = 0
total_gyro = 0

# Pega as câmeras | Câmera Plus olha mais a esquerda do total_gyro e Sub a direita, front olha a frente e reconhece vítimas
camera_front = robot.getDevice("camera1")
camera_front.enable(time_step*5)
camera_plus = robot.getDevice("camera2")
camera_plus.enable(time_step*5)
camera_sub = robot.getDevice("camera3")
camera_sub.enable(time_step*5)
colour_rgb = [0, 0, 0]
left_rgb = [0, 0, 0]
right_rgb = [0, 0, 0]
color = [0, 0]
color_p = [0, 0]
color_s = [0, 0]
processed_colour = 0
next_ground = 0
on_area_4 = False
finished_4 = False

# Pega receiver
receiver = robot.getDevice("receiver")
receiver.enable(time_step)

# Pega emmiter
emitter = robot.getDevice("emitter")

# Pega o LiDAR
lidar = robot.getDevice("lidar")
lidar.enable(time_step*5)
lidar.enablePointCloud()
point_cloud = np.zeros(512)

start = robot.getTime()


# -------------------------------------------------------------------------------


# ("walk tick", velocity, ticks) 1 block == 59 ticks
# ("walk to", target_x, target_y)
# ("collect", victim_x, victim_z)
# ("null", ticks)
# Lista de ações
action_list = [("walk_tick", 5), ("walk_back_tick", 5), ("null", 20)]
next_action = 0

current_tick = 0
tick_count = 0
dir_tick = 0

victim_list = []
victim_done = []
collected = []
victim_remove = 0

sign_sent = False
sign_action = False
identified = False
turned = False
victim_inside = False
victim_centered = False
start_send_tick = 0
start_collect_tick = 0
no_victim = False
tem_victim = False
close = False
type = 'H'
backtracking = False
temp_collect = 1700

last_pos = [0, 0]
counter = 0
qtd_prob = 0

remaining_time = 480000
score = 0

max_velocity = 5
turn_velocity = 4
speeds = [max_velocity, max_velocity]

last_4 = []

backwardX = queue.LifoQueue()
backwardY = queue.LifoQueue()
backwardX_4 = queue.LifoQueue()
backwardY_4 = queue.LifoQueue()
backwardX_save = queue.LifoQueue()
backwardY_save = queue.LifoQueue()
emergencial_backwardX = []
emergencial_backwardY = []

h = [+0, +0, +4, +2, -4, -2, +4, +2, -4, -2, +4, +2, -4, -2, +0, +0]
v = [+4, +2, +4, +2, +4, +2, +0, +0, +0, +0, -4, -2, -4, -2, -4, -2]
h_4 = [+0, +0, +0, +12, +6, +3, -12, -6, -3, +12, +6, +3, -12, -6, -3, +12, +6, +3, -12, -6, -3, +0, +0, +0]
v_4 = [+12, +6, +3, +12, +6, +3, +12, +6, +3, +0, +0, +0, +0, +0, +0, -12, -6, -3, -12, -6, -3, -12, -6, -3]

max_map = 401

parent = np.zeros(max_map*max_map*2, dtype=int)
parent = parent.reshape((max_map, max_map, 2))

parent2 = np.zeros(max_map*max_map*2, dtype=int)
parent2 = parent2.reshape((max_map, max_map, 2))

dist = np.zeros(max_map*max_map, dtype=int)
dist = dist.reshape((max_map, max_map))

initial_tick = -1
delta_angle = 0

semtempo = False
lop = False
call_tick = 0
ended = False


# -------------------------------------------------------------------------------


camera_print = False
victim_print = True
navegation_print = True
bfs_print = False
emergencial_print = True
collect_print = True


# -------------------------------------------------------------------------------
# Classes do código


# Classe representando ações individuais

class Acao:
    def __init__(self, param) -> None: # Inicialização de uma ação
        # parametros ==================================
        # walk_tick: (ticks)
        # walk_to: (x_final, y_final, x_atual, y_atual)
        # walk_back: (x_final, y_final)
        # collect: (x_vitima, y_vitima, x_atual, y_atual, list)
        # null: (ticks)

        print("init", param[0])

        if (param[0] == "null"):
            self.name = param[0]
            self.ticks = param[1]
            self.initial_coords = (0, 0)
            self.final_coords = (0, 0)
            self.v_type = 'N'
        if (param[0] == "walk_tick"):
            self.name = param[0]
            self.ticks = param[1]
            self.initial_coords = (0, 0)
            self.final_coords = (0, 0)
            self.v_type = 'N'
        if (param[0 == "walk_back_tick"]):
            self.name = param[0]
            self.ticks = param[1]
            self.initial_coords = (0, 0)
            self.final_coords = (0, 0)
            self.list = 0
        if (param[0] == "walk_to"):
            self.name = param[0]
            self.ticks = 0
            self.initial_coords = (param[3], param[4])
            self.final_coords = (param[1], param[2])
            self.v_type = 'N'
        if (param[0] == "walk_back"):
            self.name = param[0]
            self.ticks = 0
            self.initial_coords = (0, 0)
            self.final_coords = (param[1], param[2])
            self.v_type = 'N'
        if (param[0] == "collect"):
            self.name = param[0]
            self.ticks = 0
            self.initial_coords = (param[3], param[4])
            self.final_coords = (param[1], param[2])
            self.v_type = 'N'
        if (param[0] == "verify_collect"):
            self.name = param[0]
            self.ticks = 0
            self.initial_coords = (0, 0)
            self.final_coords = (param[1], param[2])
            self.v_type = param[3]

        return

    def __str__(self) -> str: # Transforma a ação em uma string para prints
        string_print = ""
        string_print += f"Tipo da ação: {self.name}\n"
        string_print += f"Tempo da ação: {self.ticks}\n"
        string_print += f"Coordenadas atuais: {self.initial_coords}\n"
        string_print += f"Coordenadas finais: {self.final_coords}\n"
        string_print += f"Tipo vítimas = {self.v_type}\n"
        return string_print

    def execute(self) -> None:  # Executa um passo
        global speeds
        global sign_sent
        global identified
        global turned
        global victim_inside
        global victim_centered
        global start_send_tick
        global start_collect_tick
        global type
        global processed_colour
        global delta_angle
        global initial_tick
        global no_victim
        global close
        global turn_velocity
        global tem_victim

        if(self.name == "walk_tick"):
            speeds = [max_velocity, max_velocity]

        if(self.name == "walk_back_tick"):
            speeds = [-max_velocity, -max_velocity]

        if(self.name == "walk_back"):   
            target_angle = math.atan2(self.final_coords[0]-last_gps[0], self.final_coords[1]-last_gps[1])
            current_angle = total_gyro
            delta_angle = target_angle - current_angle

            # virar
            if abs(delta_angle) >= 0.05:
                while delta_angle < -math.pi:
                    delta_angle = delta_angle + 2*math.pi
                while delta_angle > math.pi:
                    delta_angle = delta_angle - 2*math.pi
                if delta_angle >= 0:
                    speeds = [turn_velocity, -turn_velocity]
                else:
                    speeds = [-turn_velocity, turn_velocity]
            # andar
            else:
                speeds = [max_velocity, max_velocity]

        if(self.name == "walk_to"):  
            target_angle = math.atan2(self.final_coords[0]-last_gps[0], self.final_coords[1]-last_gps[1])
            current_angle = total_gyro
            delta_angle = target_angle - current_angle

            # virar
            if abs(delta_angle) >= 0.05:
                while delta_angle < -math.pi:
                    delta_angle = delta_angle + 2*math.pi
                while delta_angle > math.pi:
                    delta_angle = delta_angle - 2*math.pi
                if delta_angle >= 0:
                    speeds = [turn_velocity, -turn_velocity]
                else:
                    speeds = [-turn_velocity, turn_velocity]
            # andar
            else:
                if processed_colour == 0 and tick_count % 5 == 0:
                    processed_colour = 1
                    process_colour()

                speeds = [max_velocity, max_velocity]

        if(self.name == "collect"):  
            target_angle = math.atan2(self.final_coords[0]-last_gps[0], self.final_coords[1]-last_gps[1])
            current_angle = total_gyro
            delta_angle = target_angle - current_angle

            img_data = camera_front.getImage()
            img = np.array(np.frombuffer(img_data, np.uint8).reshape((camera_front.getHeight(), camera_front.getWidth(), 4)))

            colour_rgbe = [img.item(20, 0, 2), img.item(20, 0, 1), img.item(20, 0, 0)]
            colour_rgbd = [img.item(20, 63, 2), img.item(20, 63, 1), img.item(20, 63, 0)]

            if not start_collect_tick:
                start_collect_tick = tick_count

            if on_area_4 and (tick_count - start_collect_tick) < 4:
                speeds = [-max_velocity, -max_velocity]
            else:
                if abs(delta_angle) >= 0.05 and not turned:
                    while delta_angle < -math.pi:
                        delta_angle = delta_angle + 2*math.pi
                    while delta_angle > math.pi:
                        delta_angle = delta_angle - 2*math.pi
                    if delta_angle >= 0:
                        speeds = [3, -3]
                    else:
                        speeds = [-3, 3]

                elif not identified:
                    turned = True

                    # distanciar
                    if dist_lidar(0) > 0.065 and dist_lidar(30) > 0.065 and dist_lidar(490) > 0.065:
                        speeds = [5, 5]
                    elif dist_lidar(0) < 0.055:
                        speeds = [-5, -5]

                    # deixar a possível vítima totalmente dentro da câmera
                    elif not victim_inside and ((not is_wall(colour_rgbd) or not is_wall(colour_rgbe))):
                        #if collect_print:
                            #print(colour_rgbe, colour_rgbd)
                        if is_wall(colour_rgbd):
                            if collect_print:
                                print("turning left")
                            speeds = [1, -1]
                        else:
                            if collect_print:
                                print("turning right")
                            speeds = [-1, 1]

                    # centralizar ela na câmera frontal
                    elif not victim_centered:
                        victim_inside = True

                        left_camera = colour_rgbe
                        right_camera = colour_rgbd

                        left_count = 0
                        right_count = 0
                        cont = 0

                        # enquanto for parede, contar
                        for i in range(63):
                            left_camera = [img.item(20, i, 2), img.item(20, i, 1), img.item(20, i, 0)]
                            if not is_wall(left_camera):
                                if cont == 4:
                                    left_count = i - cont - 1
                                    break
                                cont = cont+1
                            else:
                                cont = 0
                        if collect_print:
                            print("left_count", left_count)

                        cont = 0

                        for i in range(63):
                            right_camera = [img.item(20, 63-i, 2), img.item(20, 63-i, 1), img.item(20, 63-i, 0)]
                            if not is_wall(right_camera):
                                if cont == 4:
                                    right_count = i - cont - 1
                                    break
                                cont = cont+1
                            else:
                                cont = 0
                        if collect_print:
                            print("right_count", right_count)

                        if (left_count == 0 and right_count == 0) or (63-left_count-right_count) < 5:
                            if collect_print:
                                print("não tem vítima para centralizar")
                            victim_centered = True
                            no_victim = True
                            speeds = [0, 0]
                        elif left_count-right_count > 2:
                            #if collect_print:
                            #   print("turning right centralizando")
                            speeds = [-0.2, 0.2]
                        elif right_count-left_count > 2:
                            #if collect_print:
                            #   print("turning left centralizando")
                            speeds = [0.2, -0.2]
                        else:
                            self.final_coords = (front_gps[0] + dist_lidar_2(0) * (math.sin(total_gyro)), front_gps[1] + dist_lidar_2(0) * (math.cos(total_gyro)))
                            victim_centered = True
                            speeds = [0, 0]

                    # identificar tipo da vítima
                    else:
                        type = identify_victim(camera_front.getImage())
                        walla, wallb = map.coords_to_tile(self.final_coords[0], self.final_coords[1])

                        if collect_print:
                            print(walla, wallb, type)

                        aux = 2*math.pi/512
                        ae = front_gps[0] + dist_lidar_2(510) * (math.sin(total_gyro+2*aux))
                        be = front_gps[1] + dist_lidar_2(510) * (math.cos(total_gyro+2*aux))
                        ad = front_gps[0] + dist_lidar_2(2) * (math.sin(total_gyro-2*aux))
                        bd = front_gps[1] + dist_lidar_2(2) * (math.cos(total_gyro-2*aux))

                        ang_min = math.atan2(ad-ae, bd-be)
                        ang_max = math.atan2(ae-ad, be-bd)

                        cv2.imwrite("victim" + "_" + str(walla) +"_" + str(wallb) + ".png", img)

                        cont = 0
                        cont2 = 0
                        t = False
                        p = False
                        for y in range(0, 40):
                            if t:
                                break
                            for x in range(0, 64):
                                colour_rgb = [img.item(y, x, 2), img.item(y, x, 1), img.item(y, x, 0)]

                                if (abs(colour_rgb[0]-133) <= 10 and abs(colour_rgb[1]-148) <= 10 and abs(colour_rgb[2]-152) <= 10):
                                    cont = cont+1
                                    # corrosive light
                                elif (abs(colour_rgb[0]-129) <= 10 and abs(colour_rgb[1]-129) <= 10 and abs(colour_rgb[2]-129) <= 10):
                                    cont = cont+1
                                    # corrosivo dark
                                elif (abs(colour_rgb[0]-169) <= 10 and abs(colour_rgb[1]-173) <= 10 and abs(colour_rgb[2]-152) <= 10):
                                    cont = cont+1
                                    cont2 = cont2+1
                                    # yellow organic light
                                elif (abs(colour_rgb[0]-164) <= 10 and abs(colour_rgb[1]-148) <= 10 and abs(colour_rgb[2]-155) <= 10):
                                    cont = cont+1
                                    cont2 = cont2+1
                                    # red organive light / flammable light
                                elif (abs(colour_rgb[0]-167) <= 10 and abs(colour_rgb[1]-161) <= 10 and abs(colour_rgb[2]-129) <= 10):
                                    cont = cont+1
                                    cont2 = cont2+1
                                    # yellow organic dark
                                elif (abs(colour_rgb[0]-162) <= 10 and abs(colour_rgb[1]-129) <= 10 and abs(colour_rgb[2]-133) <= 10):
                                    cont = cont+1
                                    cont2 = cont2+1
                                    # red organive dark / flammable dark
                                elif (abs(colour_rgb[0]-169) <= 10 and abs(colour_rgb[1]-179) <= 10 and abs(colour_rgb[2]-181) <= 10):
                                    cont = cont+1
                                    # poison light
                                elif (abs(colour_rgb[0]-166) <= 10 and abs(colour_rgb[1]-166) <= 10 and abs(colour_rgb[2]-166) <= 10):
                                    cont = cont+1
                                    # letra / poison dark
                                elif (abs(colour_rgb[0]-0) <= 5 and abs(colour_rgb[1]-0) <= 5 and abs(colour_rgb[2]-0) <= 5):
                                    t = True
                                    cont = 0
                                    break
                                    # problem corrosive
                                elif (type == 'P' and colour_rgb[0] == colour_rgb[1] and colour_rgb[1] == colour_rgb[2] and colour_rgb[0] < 130 and abs(colour_rgb[0]-10) > 5 and abs(colour_rgb[0]-30) > 5):
                                    p = True
                                    t = True
                                    cont = 0
                                    break
                                    # problem poison

                        if collect_print:
                            print("cont igual:", cont, cont2)
                        if (type == 'P' and p == False and cont2 < 100) or cont > 170 or (cont > 140 and ((abs(total_gyro-ang_min) < 1 or 2*math.pi-abs(total_gyro-ang_min) < 1) or (abs(total_gyro-ang_max) < 1 or 2*math.pi-abs(total_gyro-ang_max) < 1))):
                            if collect_print:
                                print("vitima já foi pega")
                            no_victim = True

                        if type == 'N' or no_victim:
                            sign_sent = True
                            print("vítima falsa")
                            action_list.insert(1, ("walk_back_tick", 5))
                            action_list.insert(2, ("walk_back", self.initial_coords[0], self.initial_coords[1]))
                        else:
                            if collect_print:
                                print("wall", walla, wallb)
                            if walla % 2 == 0 and (abs(ang_min - math.pi/2) < 0.5 or abs(ang_min + math.pi/2) < 0.5):
                                if abs(map.tile_to_coords(walla-1, wallb)[0] - self.final_coords[0]) < abs(map.tile_to_coords(walla+1, wallb)[0] - self.final_coords[0]):
                                    walla = walla-1
                                else:
                                    walla = walla+1
                                if collect_print:
                                    print("muda walla")
                            if wallb % 2 == 0 and (abs(ang_min - 0) < 0.5 or abs(ang_min - math.pi) < 0.5 or abs(ang_min + math.pi) < 0.5):
                                if abs(map.tile_to_coords(walla, wallb-1)[1] - self.final_coords[1]) < abs(map.tile_to_coords(walla, wallb+1)[1] - self.final_coords[1]):
                                    wallb = wallb-1
                                else:
                                    wallb = wallb+1
                                if collect_print:
                                    print("muda wallb")

                            if walla % 2 == wallb % 2:
                                if abs(ang_min + 3*math.pi/4) < math.pi/5:
                                    if abs(map.tile_to_coords(walla-1, wallb)[0] - self.final_coords[0]) < abs(map.tile_to_coords(walla, wallb+1)[1] - self.final_coords[1]):
                                        walla = walla-1
                                        if collect_print:
                                            print("- walla 1")
                                    else:
                                        wallb = wallb+1
                                        if collect_print:
                                            print("+ wallb 1")
                                if abs(ang_min - 3*math.pi/4) < math.pi/5:
                                    if abs(map.tile_to_coords(walla-1, wallb)[0] - self.final_coords[0]) < abs(map.tile_to_coords(walla, wallb-1)[1] - self.final_coords[1]):
                                        walla = walla-1
                                        if collect_print:
                                            print("- walla 2")
                                    else:
                                        wallb = wallb-1
                                        if collect_print:
                                            print("- wallb 2")
                                if abs(ang_min + math.pi/4) < math.pi/5:
                                    if abs(map.tile_to_coords(walla+1, wallb)[0] - self.final_coords[0]) < abs(map.tile_to_coords(walla, wallb+1)[1] - self.final_coords[1]):
                                        walla = walla+1
                                        if collect_print:
                                            print("+ walla 3")
                                    else:
                                        wallb = wallb+1
                                        if collect_print:
                                            print("+ wallb 3")
                                if abs(ang_min - math.pi/4) < math.pi/5:
                                    if abs(map.tile_to_coords(walla+1, wallb)[0] - self.final_coords[0]) < abs(map.tile_to_coords(walla, wallb-1)[1] - self.final_coords[1]):
                                        walla = walla+1
                                        if collect_print:
                                            print("+ walla 4")
                                    else:
                                        wallb = wallb-1
                                        if collect_print:
                                            print("- wallb 4")

                            if not on_area_4:
                                collected.append([type, walla, wallb])
                            if collect_print:
                                print(type)
                            identified = True

                elif dist_lidar(0) > 0.053 and dist_lidar(50) > 0.053 and dist_lidar(462) > 0.053 and start_send_tick == 0:
                    speeds = [5, 5]

                # manda a vítima
                elif not sign_sent:
                    if not start_send_tick:
                        print("começa contagem")
                        start_send_tick = tick_count

                    speeds = [0, 0]
                    victim_type = bytes(type, "utf-8")

                    if (tick_count - start_send_tick)*16 > temp_collect:
                        if collect_print:
                            print("Mandei Vítima!!", tick_count)
                        sign_sent = True
                        message = struct.pack("i i c", int((self.final_coords[0]+initial_gps[0])*100), int((self.final_coords[1]+initial_gps[1])*100), victim_type)
                        emitter.send(message)
                        action_list.insert(next_action+1, ("null", 60))
                        action_list.insert(next_action+2, ("verify_collect", self.final_coords[0], self.final_coords[1], type))

                        if abs(math.sqrt(pow(self.initial_coords[0]-last_gps[0], 2)+pow(self.initial_coords[1]-last_gps[1], 2))) > 0.005:
                            if not on_area_4:
                                action_list.insert(next_action+3, ("walk_back_tick", 5))
                                action_list.insert(next_action+4, ("walk_back", self.initial_coords[0], self.initial_coords[1]))
                            else:
                                action_list.insert(next_action+3, ("walk_back", self.initial_coords[0], self.initial_coords[1]))

        if(self.name == "verify_collect"):
            img_data = camera_front.getImage()
            img = np.array(np.frombuffer(img_data, np.uint8).reshape((camera_front.getHeight(), camera_front.getWidth(), 4)))
            cv2.imwrite("verify" + "_" + str(self.v_type) + ".png", img)

            aux = 2*math.pi/512
            ae = front_gps[0] + dist_lidar_2(510) * (math.sin(total_gyro+2*aux))
            be = front_gps[1] + dist_lidar_2(510) * (math.cos(total_gyro+2*aux))
            ad = front_gps[0] + dist_lidar_2(2) * (math.sin(total_gyro-2*aux))
            bd = front_gps[1] + dist_lidar_2(2) * (math.cos(total_gyro-2*aux))

            ang_min = math.atan2(ad-ae, bd-be)
            ang_max = math.atan2(ae-ad, be-bd)

            cont = 0
            t = False
            p = False
            for y in range(0, 40):
                if t:
                    break
                for x in range(0, 64):
                    colour_rgb = [img.item(y, x, 2), img.item(y, x, 1), img.item(y, x, 0)]

                    if (abs(colour_rgb[0]-133) <= 10 and abs(colour_rgb[1]-148) <= 10 and abs(colour_rgb[2]-152) <= 10):
                        cont = cont+1
                        # corrosive light
                    elif (abs(colour_rgb[0]-129) <= 10 and abs(colour_rgb[1]-129) <= 10 and abs(colour_rgb[2]-129) <= 10):
                        cont = cont+1
                        # corrosivo dark
                    elif (abs(colour_rgb[0]-169) <= 10 and abs(colour_rgb[1]-173) <= 10 and abs(colour_rgb[2]-152) <= 10):
                        cont = cont+1
                        # yellow organic light
                    elif (abs(colour_rgb[0]-164) <= 10 and abs(colour_rgb[1]-148) <= 10 and abs(colour_rgb[2]-155) <= 10) or (abs(colour_rgb[0]-199) <= 10 and abs(colour_rgb[1]-128) <= 10 and abs(colour_rgb[2]-140) <= 10):
                        cont = cont+1
                        # red organive light / flammable light
                    elif (abs(colour_rgb[0]-167) <= 10 and abs(colour_rgb[1]-161) <= 10 and abs(colour_rgb[2]-129) <= 10):
                        cont = cont+1
                        # yellow organic dark
                    elif (abs(colour_rgb[0]-162) <= 10 and abs(colour_rgb[1]-129) <= 10 and abs(colour_rgb[2]-133) <= 10):
                        cont = cont+1
                        # red organive dark / flammable dark
                    elif (abs(colour_rgb[0]-169) <= 10 and abs(colour_rgb[1]-179) <= 10 and abs(colour_rgb[2]-181) <= 10):
                        cont = cont+1
                        # poison light
                    elif (abs(colour_rgb[0]-166) <= 10 and abs(colour_rgb[1]-166) <= 10 and abs(colour_rgb[2]-166) <= 10):
                        cont = cont+1
                        # letra / poison dark
                    elif (abs(colour_rgb[0]-0) <= 5 and abs(colour_rgb[1]-0) <= 5 and abs(colour_rgb[2]-0) <= 5):
                        t = True
                        cont = 0
                        break
                        # problem corrosive
                    elif (self.v_type == 'P' and colour_rgb[0] == colour_rgb[1] and colour_rgb[1] == colour_rgb[2] and colour_rgb[0] < 130 and abs(colour_rgb[0]-10) > 5 and abs(colour_rgb[0]-30) > 5):
                        p = True
                        t = True
                        cont = 0
                        break
                        # problem poison


            print("cont igual:",cont)
            print(total_gyro-ang_min, ang_max-total_gyro)
            
            if (self.v_type == 'P' and p) or cont < 50 or (cont < 140 and self.v_type != 'H' and self.v_type != 'U' and self.v_type != 'S' and self.v_type != 'C') or (cont < 170 and not ((abs(total_gyro-ang_min) < 1 or 2*math.pi-abs(total_gyro-ang_min) < 1) or (abs(total_gyro-ang_max) < 1 or 2*math.pi-abs(total_gyro-ang_max) < 1))) or (cont < 100 and (self.v_type != 'H' and self.v_type != 'U' and self.v_type != 'S' and self.v_type != 'C') and ((abs(total_gyro-ang_min) < 1 or 2*math.pi-abs(total_gyro-ang_min) < 1) or (abs(total_gyro-ang_max) < 1 or 2*math.pi-abs(total_gyro-ang_max) < 1))):
                print("enviando denovo pq burrei")
                victim_type = bytes(self.v_type, "utf-8")
                message = struct.pack("i i c", int((self.final_coords[0]+initial_gps[0])*100), int((self.final_coords[1]+initial_gps[1])*100), victim_type)
                emitter.send(message)

            sign_sent = True

        if(self.name == "null"):
            speeds = [0, 0]

        return

    def is_done(self) -> bool:  # Testa se a ação end
        global processed_colour
        global next_ground
        global hard_pos
        global action_list
        global next_action
        global sign_sent
        global sign_action
        global identified
        global turned
        global victim_inside
        global victim_centered
        global initial_tick
        global no_victim
        global last_pos
        global qtd_prob
        global counter
        global tem_victim
        global start_send_tick
        global start_collect_tick

        if(self.name == "walk_tick"):
            if current_tick > self.ticks:
                return True

        if(self.name == "walk_back_tick"):
            if current_tick > self.ticks:
                return True

        if(self.name == "walk_back"):
            if abs(math.sqrt(pow(self.final_coords[0]-last_gps[0], 2)+pow(self.final_coords[1]-last_gps[1], 2))) <= 0.005:
                counter = 0
                return True

            if(counter == 300):
                print("problemaaaaaaa walk back")
                counter = 0
                action_list = [["null", 2]]
                action_list.append(("walk_back_tick", 15))
                return True

            if dist_coords(last_pos[0], last_pos[1], last_gps[0], last_gps[1]) > 0.02:
                last_pos = [last_gps[0], last_gps[1]]
                counter = 0
            else:
                counter = counter+1

        elif(self.name == "walk_to"):
            x, y = map.coords_to_tile(self.final_coords[0], self.final_coords[1])
            x4, y4 = map4.coords_to_tile(self.final_coords[0], self.final_coords[1])
            myX, myY = map.coords_to_tile(self.initial_coords[0], self.initial_coords[1])
            myX4, myY4 = map4.coords_to_tile(self.initial_coords[0], self.initial_coords[1])

            if(counter == 300):
                qtd_prob = qtd_prob + 2
                print("problemaaaaaaaaaaaaaaaaaa")
                #if on_area_4:
                #    for k in range(-1, 2):
                #        for l in range(-1, 2):
                #            map4.mark[y4+k, x4+l] = -1
                #map.mark[y, x] = -1
                coordX = front_gps[0] + 0.015 * math.sin(total_gyro)
                coordY = front_gps[1] + 0.015 * math.cos(total_gyro)
                print(last_gps[0], last_gps[1])
                print(coordX, coordY)
                map.matrix[map.coords_to_tile(coordX, coordY)[1], map.coords_to_tile(coordX, coordY)[0]] = -1 
                map4.matrix[map4.coords_to_tile(coordX, coordY)[1], map4.coords_to_tile(coordX, coordY)[0]] = -1 
                counter = 0
                action_list = [["null", 2]]
                action_list.append(("walk_back_tick", 15))
                action_list.append(("walk_back", self.initial_coords[0],self.initial_coords[1]))
                if backtracking:
                    aux = False
                    while(not backwardX_save.empty()):
                        x, y = backwardX_save.get(), backwardY_save.get()
                        if on_area_4:
                                if myX4 == x and myY4 == y:
                                    aux = True
                                if aux:
                                    backwardX_4.put(x)
                                    backwardY_4.put(y)
                        else:
                                if myX == x and myY == y:
                                    aux = True
                                if aux:
                                    backwardX.put(x)
                                    backwardY.put(y)
                return True

            if dist_coords(last_pos[0], last_pos[1], last_gps[0], last_gps[1]) > 0.02:
                last_pos = [last_gps[0], last_gps[1]]
                counter = 0
            else:
                counter = counter+1            

            if next_ground == 2:
                qtd_prob = qtd_prob+1
                processed_colour = False
                next_ground = 0
                counter = 0
                map.mark[y, x] = -1
                map4.mark[y4, x4] = -1
                action_list = [["null", 2]]
                action_list.append(("walk_back_tick", 5))
                print("buraco")
                return True

            if not on_area_4:
                if not backtracking and not verify_wall(myX, myY, x, y):
                    processed_colour = False
                    counter = 0
                    action_list = [["null", 2]]
                    action_list.append(("walk_back_tick", 5))
                    action_list.append(("walk_back", self.initial_coords[0], self.initial_coords[1]))
                    print("2")
                    return True
            else:
                if not backtracking and not verify_wall_4(myX4, myY4, x4, y4):
                    processed_colour = False
                    counter = 0
                    action_list = [["null", 2]]
                    action_list.append(("walk_back_tick", 5))
                    print("paro paro onde quero ir")
                    return True

            if abs(math.sqrt(pow(self.final_coords[0]-last_gps[0], 2)+pow(self.final_coords[1]-last_gps[1], 2))) <= 0.005:
                processed_colour = False
                counter = 0
                qtd_prob = 0
                print("1")
                return True

        elif(self.name == "collect"):
            if dist_coords(last_gps[0], last_gps[1], self.initial_coords[0], self.initial_coords[1]) > 0.1:
                print("andou muito")
                sign_sent = True
                action_list = [["null", 2]]
                action_list.append(("walk_back_tick", 15))
                action_list.append(("walk_back", self.initial_coords[0],self.initial_coords[1]))

            if(counter == 625):
                print("problemaaaaaaa colect")
                counter = 0
                if identified and start_send_tick == 0:
                    start_send_tick = 1
                else:
                    sign_sent = True
                    action_list = [["null", 2]]
                    if turned:
                        action_list.append(("walk_back_tick", 15))
                        action_list.append(("walk_back", self.initial_coords[0], self.initial_coords[1]))
                    else:
                        action_list.append(("walk_back_tick", 7))
                        action_list.append(('collect', self.final_coords[0], self.final_coords[1], self.initial_coords[0], self.initial_coords[1]))

            if dist_coords(last_pos[0], last_pos[1], last_gps[0], last_gps[1]) > 0.02:
                last_pos = [last_gps[0], last_gps[1]]
                counter = 0
            else:
                counter = counter+1

            if sign_sent:
                print("acabaa")
                sign_sent = False
                start_send_tick = 0
                sign_action = False
                identified = False
                turned = False
                victim_inside = False
                victim_centered = False
                start_collect_tick = 0
                no_victim = False
                close = False
                counter = 0
                tem_victim = False
                return True

        elif(self.name == "verify_collect"):
            if sign_sent:
                sign_sent = False
                return True

        elif(self.name == "null"):
            if current_tick > self.ticks:
                return True

        return False


# Classe representando o mapa

class Map:
    def __init__(self):
        self.matrix = np.zeros(max_map*max_map, dtype=int)
        self.matrix = self.matrix.reshape((max_map, max_map))

        self.mark = np.zeros(max_map*max_map, dtype=int)
        self.mark = self.mark.reshape((max_map, max_map))

        self.mark_1 = np.zeros(max_map*max_map)
        self.mark_1 = self.mark_1.reshape((max_map, max_map))
        self.mark_2 = np.zeros(max_map*max_map)
        self.mark_2 = self.mark_2.reshape((max_map, max_map))

        self.yMin = max_map+1
        self.xMin = max_map+1
        self.yMax = -1
        self.xMax = -1

    def coords_to_tile(self, coordX, coordY):
        mapX = int((max_map-1)/2 + round(coordX/0.03))
        mapY = int((max_map-1)/2 + round(coordY/0.03))

        return mapX, mapY

    def coords_to_wall(self, coordX, coordY):
        mapX, mapY = self.coords_to_tile(coordX, coordY)
        invX, invY = self.tile_to_coords(mapX, mapY)

        if abs(coordY-invY) > 0.004 and abs(coordX-invX) > 0.004:
            return -1, -1
        else:
            return mapX, mapY

    def coords_to_tile_center(self, coordX, coordY):
        mapX = int((max_map-1)/2 + 4 * round((coordX)/0.12))
        mapY = int((max_map-1)/2 + 4 * round((coordY)/0.12))

        if (mapY) % 4 == 0 and (mapX) % 4 == 0:
            return mapX, mapY
        else:
            return -1, -1

    def coords_to_even_tile(self, coordX, coordY):
        mapX = int((max_map-1)/2 + 2 * round((coordX)/0.06))
        mapY = int((max_map-1)/2 + 2 * round((coordY)/0.06))

        if (mapY) % 2 == 0 and (mapX) % 2 == 0:
            return mapX, mapY
        else:
            return -1, -1

    def tile_to_coords(self, tileX, tileY):
        coordX = (tileX - (max_map-1)/2) * 0.03
        coordY = (tileY - (max_map-1)/2) * 0.03

        return coordX, coordY

    def add_wall(self, coordX, coordY):
        mapX, mapY = self.coords_to_wall(coordX, coordY)
        if mapY != -1 and mapX != -1:
            self.update_extremes(coordX, coordY)
            self.mark[mapY, mapX] = 2
            if self.matrix[mapY][mapX] <= 0 or self.matrix[mapY][mapX] == 10:
                if (mapX % 2 != 1 or mapY % 2 != 1):
                    self.matrix[mapY][mapX] = 1
                else:
                    if ((self.matrix[mapY+1][mapX] and self.matrix[mapY+1][mapX-1] and self.matrix[mapY][mapX+1] and self.matrix[mapY-1][mapX+1]) or (self.matrix[mapY-1][mapX] and self.matrix[mapY-1][mapX-1] and self.matrix[mapY][mapX+1] and self.matrix[mapY+1][mapX+1]) or (self.matrix[mapY+1][mapX] and self.matrix[mapY+1][mapX+1] and self.matrix[mapY][mapX-1] and self.matrix[mapY-1][mapX-1]) or (self.matrix[mapY-1][mapX] and self.matrix[mapY-1][mapX+1] and self.matrix[mapY][mapX-1] and self.matrix[mapY+1][mapX-1])):
                        self.matrix[mapY][mapX] = 0
                    else:
                        self.matrix[mapY][mapX] = -1

                if on_area_4:
                    self.matrix[mapY][mapX] = 10
                    

    def add_victim(self, letra, mapX, mapY):
        # if(mapX % 2 == 1):
        #    if((mapX+1) % 4 == 0):
        #        mapX = mapX+1
        #    else:
        #        mapX = mapX-1

        # if(mapY % 2 == 1):
        #    if((mapY+1) % 4 == 0):
        #        mapY = mapY+1
        #    else:
        #        mapY = mapY-1

        if self.matrix[mapY][mapX] == 1:
            self.matrix[mapY][mapX] = ord(letra)
        else:
            self.matrix[mapY][mapX] = int(
                str(self.matrix[mapY][mapX]) + str(ord(letra)))

    def update_extremes(self, coordX, coordY):
        mapX, mapY = self.coords_to_wall(coordX, coordY)
        if mapX != -1 and mapY != -1:
            if mapY < self.yMin:
                self.yMin = mapY
            if mapY > self.yMax:
                self.yMax = mapY
            if mapX < self.xMin:
                self.xMin = mapX
            if mapX > self.xMax:
                self.xMax = mapX

    def print_map(self):
        yMin = self.yMin
        yMax = self.yMax
        xMin = self.xMin
        xMax = self.xMax
        if(yMin != max_map+1):
            #x,y = map.coords_to_tile(last_gps[0], last_gps[1])

            #for y in range(yMin, yMax+1):
            #    for x in range(xMin, xMax+1):
            #        print(str(self.matrix[y, x])+' ', end="")
            #    print(' ')

            # if tick_count % 20 == 0:
            s = 'teste' + str(tick_count) + '.png'
            M = abs(self.matrix[yMin:yMax+1, xMin:xMax+1])*255
            img = cv2.imwrite(s, M)

    def cleanup_matrix(self):
        yMin = (self.yMin//4)*4-8
        yMax = (self.yMax//4)*4+4
        xMin = (self.xMin//4)*4-8
        xMax = (self.xMax//4)*4+4
        if(yMin != max_map+1):
            M = self.matrix[yMin:yMax+1, xMin:xMax+1]
            for i in range(0, M.shape[0]-1, 4):
                for j in range(0, M.shape[1]-1, 4):
                    if(not M[i][j] and (M[i+1][j]+M[i-1][j]+M[i][j-1]+M[i][j+1]) <= 1):
                        M[i+1][j], M[i-1][j], M[i][j-1], M[i][j+1] = 0, 0, 0, 0

    def end_map(self):
        global ended
        ended = True

        temporary_array = []
        final_map = []

        # linhas e colunas erradas
        tem_mapa = False
        for i in range(self.yMin, self.yMax+1):
            tem_parede = False
            for j in range(self.xMin, self.xMax+1):
                if self.matrix[i, j] != 0:
                    tem_parede = True
                    tem_mapa = True
            if not tem_parede:
                if not tem_mapa:
                    self.yMin = self.yMin + 1
                else:
                    self.yMax = self.yMax - 1
        tem_mapa = False
        for i in range(self.xMin, self.xMax+1):
            tem_parede = False
            for j in range(self.yMin, self.yMax+1):
                if self.matrix[j, i] != 0:
                    tem_parede = True
                    tem_mapa = True
            if not tem_parede:
                if not tem_mapa:
                    self.xMin = self.xMin + 1
                else:
                    self.xMax = self.xMax - 1

        # paredes que deveriam existir
        
        '''
        colunaMin = np.zeros(self.xMax, dtype=int)
        colunaMax = np.zeros(self.xMax, dtype=int)
        for i in range(self.yMin+2, self.yMax-1, 4):
            linhaMin, linhaMax = 0, 0
            for j in range(self.xMin+2, self.xMax-1, 4):
                if self.mark[i, j] != 0 or self.matrix[i-1, j-1] != 0 or self.matrix[i-1, j] != 0 or self.matrix[i-1, j+1] != 0 or self.matrix[i, j-1] != 0 or self.matrix[i, j] != 0 or self.matrix[i, j+1] != 0 or self.matrix[i+1, j-1] != 0 or self.matrix[i+1, j] != 0 or self.matrix[i+1, j+1] != 0:
                    if not linhaMin:
                        linhaMin = j
                        if self.matrix[i-1, j-2] == 0:
                            self.matrix[i-1, j-2] = 1
                        if self.matrix[i, j-2] == 0:
                            self.matrix[i, j-2] = 1
                        if self.matrix[i+1, j-2] == 0:
                            self.matrix[i+1, j-2] = 1
                    linhaMax = j

                    if not colunaMin[j]:
                        colunaMin[j] = i
                        if self.matrix[i-2, j-1] == 0:
                            self.matrix[i-2, j-1] = 1
                        if self.matrix[i-2, j] == 0:
                            self.matrix[i-2, j] = 1
                        if self.matrix[i-2, j+1] == 0:
                            self.matrix[i-2, j+1] = 1
                    colunaMax[j] = i

                if i == self.yMax-2:
                    if self.matrix[colunaMax[j]+2, j-1] == 0:
                        self.matrix[colunaMax[j]+2, j-1] = 1
                    if self.matrix[colunaMax[j]+2, j] == 0:
                        self.matrix[colunaMax[j]+2, j] = 1
                    if self.matrix[colunaMax[j]+2, j+1] == 0:
                        self.matrix[colunaMax[j]+2, j+1] = 1
            if self.matrix[i-1, linhaMax+2] == 0:
                self.matrix[i-1, linhaMax+2] = 1
            if self.matrix[i, linhaMax+2] == 0:
                self.matrix[i, linhaMax+2] = 1
            if self.matrix[i+1, linhaMax+2] == 0:
                self.matrix[i+1, linhaMax+2] = 1
        '''

        # apaga obstáculos
        for i in range(self.yMin, self.yMax):
            for j in range(self.xMin, self.xMax):
                if self.matrix[i, j] == -1:
                    self.matrix[i, j] = 0

        for i in range(self.yMin+1, self.yMax):
            for j in range(self.xMin+1, self.xMax):
                if (i % 2 == 1 or j % 2 == 1) and self.matrix[i, j] == 1:
                    # casas impar nunca tem parede | solitário
                    if (i % 2 == 1 and j % 2 == 1) or (self.matrix[i-1, j] <= 0 and self.matrix[i+1, j] <= 0 and self.matrix[i, j-1] <= 0 and self.matrix[i, j+1] <= 0):
                        print(i, j, "impar ou solo")
                        self.matrix[i, j] = 0
                    else:
                        # se tem erro o espaço todo é um obs. ent pinta
                        if ((i-1) % 2 != 1 or j % 2 != 1) and ((self.matrix[i-1, j] > 10 or self.matrix[i-1, j] == 1) and self.matrix[i+1, j] == 0):
                            if not (self.matrix[i+1, j-1] == 1 and self.matrix[i+1, j-2] == 1) and not (self.matrix[i+1, j+1] == 1 and self.matrix[i+1, j+2] == 1):
                                print(i, j, "1")
                                for k in range(-1, 2):
                                    for l in range(-1, 2):
                                        if self.matrix[i-1+k, j+l] == 1 or self.matrix[i-1+k, j+l] == 2:
                                            self.matrix[i-1+k, j+l] = 0
                        if ((i+1) % 2 != 1 or j % 2 != 1) and (self.matrix[i-1, j] == 0 and (self.matrix[i+1, j] > 10 or self.matrix[i+1, j] == 1)):
                            if not (self.matrix[i-1, j-1] == 1 and self.matrix[i-1, j-2] == 1) and not (self.matrix[i-1, j+1] == 1 and self.matrix[i-1, j+2] == 1):
                                print(i, j, "2")
                                for k in range(-1, 2):
                                    for l in range(-1, 2):
                                        if self.matrix[i+1+k, j+l] == 1 or self.matrix[i+1+k, j+l] == 2:
                                            self.matrix[i+1+k, j+l] = 0
                        if (i % 2 != 1 or (j-1) % 2 != 1) and ((self.matrix[i, j-1] > 10 or self.matrix[i, j-1] == 1) and self.matrix[i, j+1] == 0):
                            if not (self.matrix[i-1, j+1] == 1 and self.matrix[i-2, j+1] == 1) and not (self.matrix[i+1, j+1] == 1 and self.matrix[i+2, j+1] == 1):
                                print(i, j, "3")
                                for k in range(-1, 2):
                                    for l in range(-1, 2):
                                        if self.matrix[i+k, j-1+l] == 1 or self.matrix[i+k, j-1+l] == 2:
                                            self.matrix[i+k, j-1+l] = 0
                        if (i % 2 != 1 or (j+1) % 2 != 1) and (self.matrix[i, j-1] == 0 and (self.matrix[i, j+1] > 10 or self.matrix[i, j+1] == 1)):
                            if not (self.matrix[i-1, j-1] == 1 and self.matrix[i-2, j-1] == 1) and not (self.matrix[i+1, j-1] == 1 and self.matrix[i+2, j-1] == 1):
                                print(i, j, "4")
                                for k in range(-1, 2):
                                    for l in range(-1, 2):
                                        if self.matrix[i+k, j+1+l] == 1 or self.matrix[i+k, j+1+l] == 2:
                                            self.matrix[i+k, j+1+l] = 0

        for [t, a, b] in collected:
            map.add_victim(t, a, b)

        for i in range(self.yMin, self.yMax+1):
            for j in range(self.xMin, self.xMax+1):
                if (self.matrix[i, j] > 10 or self.matrix[i, j] == 1) and (self.matrix[i-1, j] == 0 and self.matrix[i+1, j] == 0 and self.matrix[i, j-1] == 0 and self.matrix[i, j+1] == 0):
                    self.matrix[i, j] = 0
                if self.matrix[i, j] == 10:
                    self.matrix[i, j] = 1

                append_str = str(int(self.matrix[i, j]))
                if len(str(self.matrix[i, j])) > 1:  # é uma letra
                    if len(str(self.matrix[i, j])) == 2:
                        append_str = chr(self.matrix[i, j])
                    else:  # são duas letras juntas
                        append_str = chr(int(str(self.matrix[i, j])[0] + str(self.matrix[i, j])[1])) + chr(
                            int(str(self.matrix[i, j])[2] + str(self.matrix[i, j])[3]))
                temporary_array.append(append_str)
                print(append_str + ' ', end="")
            final_map.append(temporary_array.copy())
            temporary_array.clear()
            print(' ')

        #print(" ")

        # final_map.append(temporary_array.copy())
        definitive_map = np.array(final_map)

        d = definitive_map.shape
        d_bytes = struct.pack('2i', *d)
        flatMap = ','.join(definitive_map.flatten())
        dub_bytes = flatMap.encode('utf-8')
        final_bytes = d_bytes + dub_bytes
        emitter.send(final_bytes)
        map_evaluate_request = struct.pack('c', b'M')
        emitter.send(map_evaluate_request)

        if map.coords_to_tile(0, 0) == map.coords_to_tile(last_gps[0], last_gps[1]):
            exit_mes = struct.pack('c', b'E')
            emitter.send(exit_mes)


map = Map()
map4 = Map()


class Map4:
    def __init__(self):
        self.initial_4 = [last_gps[0], last_gps[1]]

        self.matrix = np.zeros(max_map*max_map, dtype=int)
        self.matrix = self.matrix.reshape((max_map, max_map))

        self.mark = np.zeros(max_map*max_map, dtype=int)
        self.mark = self.mark.reshape((max_map, max_map))

        self.mark_V = np.zeros(max_map*max_map, dtype=int)
        self.mark_V = self.mark.reshape((max_map, max_map))

        self.yMin = max_map+1
        self.xMin = max_map+1
        self.yMax = -1
        self.xMax = -1

    def coords_to_tile(self, coordX, coordY):
        mapX = int((max_map-1)/2 + round((coordX-self.initial_4[0])/0.01))
        mapY = int((max_map-1)/2 + round((coordY-self.initial_4[1])/0.01))

        return mapX, mapY

    def coords_to_walk_tile(self, coordX, coordY):
        mapX = int((max_map-1)/2 + 3 * round((coordX-self.initial_4[0])/0.03))
        mapY = int((max_map-1)/2 + 3 * round((coordY-self.initial_4[1])/0.03))

        return mapX, mapY

    def coords_to_tile_center(self, coordX, coordY):
        mapX = int((max_map-1)/2 + 12 * round((coordX-self.initial_4[0])/0.12))
        mapY = int((max_map-1)/2 + 12 * round((coordY-self.initial_4[1])/0.12))

        if (mapY) % 4 == 0 and (mapX) % 4 == 0:
            return mapX, mapY
        else:
            return -1, -1

    def coords_to_even_tile(self, coordX, coordY):
        mapX = int((max_map-1)/2 + 6 * round((coordX-self.initial_4[0])/0.06))
        mapY = int((max_map-1)/2 + 6 * round((coordY-self.initial_4[1])/0.06))

        if (mapY) % 2 == 0 and (mapX) % 2 == 0:
            return mapX, mapY
        else:
            return -1, -1

    def tile_to_coords(self, tileX, tileY):
        coordX = (tileX - (max_map-1)/2) * 0.01 + self.initial_4[0]
        coordY = (tileY - (max_map-1)/2) * 0.01 + self.initial_4[1]

        return coordX, coordY

    def add_wall(self, coordX, coordY):
        mapX, mapY = self.coords_to_tile(coordX, coordY)
        map.update_extremes(coordX, coordY)
        map4.update_extremes(coordX, coordY)
        if self.matrix[mapY][mapX] <= 0:
            self.mark[mapY, mapX] = 2
            self.matrix[mapY][mapX] = 1

    def add_victim(self, letra, mapX, mapY):
        # if(mapX % 2 == 1):
        #    if((mapX+1) % 4 == 0):
        #        mapX = mapX+1
        #    else:
        #        mapX = mapX-1

        # if(mapY % 2 == 1):
        #    if((mapY+1) % 4 == 0):
        #        mapY = mapY+1
        #    else:
        #        mapY = mapY-1

        if self.matrix[mapY][mapX] == 1:
            self.matrix[mapY][mapX] = ord(letra)
        else:
            self.matrix[mapY][mapX] = int(
                str(self.matrix[mapY][mapX]) + str(ord(letra)))

    def update_extremes(self, coordX, coordY):
        mapX, mapY = self.coords_to_tile(coordX, coordY)
        if mapY < self.yMin:
            self.yMin = mapY
        if mapY > self.yMax:
            self.yMax = mapY
        if mapX < self.xMin:
            self.xMin = mapX
        if mapX > self.xMax:
            self.xMax = mapX

    def print_map(self):
        yMin = self.yMin
        yMax = self.yMax
        xMin = self.xMin
        xMax = self.xMax
        if(yMin != max_map+1):
            #    print(self.matrix[xMin:xMax+1,yMin:yMax+1])

            # if tick_count % 20 == 0:
            s = 'teste' + str(tick_count) + '_4.png'
            M = self.matrix[yMin:yMax+1, xMin:xMax+1]*255
            mapx, mapy = self.coords_to_tile(last_gps[0], last_gps[1])
            for i in range(7):
                for j in range(7):
                    if(mapy+3-i-yMin < yMax-yMin and mapy+3-i-yMin >= 0 and mapx+3-j-xMin >= 0 and mapx+3-j-xMin < yMax-yMin):
                        M[mapy+3-i-yMin, mapx+3-j-xMin] = 170
            img = cv2.imwrite(s, M)

    def cleanup_matrix(self):
        yMin = (self.yMin)-2
        yMax = (self.yMax)+1
        xMin = (self.xMin)-2
        xMax = (self.xMax)+1
        if(yMin != max_map+1):
            M = self.matrix[yMin:yMax+1, xMin:xMax+1]
            for i in range(0, M.shape[0]-1, 1):
                for j in range(0, M.shape[1]-1, 1):
                    if(not M[i][j] and (M[i+1][j]+M[i-1][j]+M[i][j-1]+M[i][j+1]) < 1):
                        #M[i+1][j],M[i-1][j],M[i][j-1],M[i][j+1] = 0,0,0,0
                        M[i][j] = 0


# -------------------------------------------------------------------------------
# Funções de sensores


def update_sensors():
    global tick_count
    global point_cloud
    global max_velocity
    global turn_velocity
    global total_gyro
    global lop
    global action_list
    global next_action
    global dir_tick
    global current_tick
    global lop_gps

    if tick_count == 0:
        initial_gps[0:2] = [gps.getValues()[0], gps.getValues()[2]]
        a, b = map.coords_to_tile_center(0, 0)
        map.matrix[b-1, a-1] = 5
        map.matrix[b-1, a+1] = 5
        map.matrix[b+1, a-1] = 5
        map.matrix[b+1, a+1] = 5

    if abs(gps.getValues()[0] - initial_gps[0] - last_gps[0]) > 0.03 or abs(gps.getValues()[2] - initial_gps[1] - last_gps[1]) > 0.03:
        lop = True
        lop_gps = [gps.getValues()[0] - initial_gps[0], gps.getValues()[2] - initial_gps[1]]
        print("deu ruim")
        action_list = [("walk_tick", 5), ("walk_back_tick", 5), ("null", 20)]
        next_action = 0
        current_tick = 0
        dir_tick = 0

        a, b = map.coords_to_tile(last_gps[0], last_gps[1])
        a4, b4 = map4.coords_to_tile(last_gps[0], last_gps[1])
        if on_area_4:
            for k in range(-1, 2):
                for l in range(-1, 2):
                    map4.matrix[b4+k, a4+l] = -1
        for m in range(-1, 2): # Marca todos como parede
            for n in range(-1, 2):
                map.matrix[b+m, a+n] = -1
        print("coloca parede falsa", a, b)

        if backtracking:
            aux = False
            while(not backwardX_save.empty()):
                x, y = backwardX_save.get(), backwardY_save.get()
                if on_area_4:
                        if a4 == x and b4 == y:
                            aux = True
                        if aux:
                            backwardX_4.put(x)
                            backwardY_4.put(y)
                else:
                        if a == x and b == y:
                            aux = True
                        if aux:
                            backwardX.put(x)
                            backwardY.put(y)

    last_gps[0] = gps.getValues()[0] - initial_gps[0]
    last_gps[1] = gps.getValues()[2] - initial_gps[1]

    front_gps[0] = last_gps[0] + 0.03284 * math.sin(total_gyro)
    front_gps[1] = last_gps[1] + 0.03284 * math.cos(total_gyro)

    if (dir_tick < 5):
        pass
    elif (dir_tick == 5):
        total_gyro = math.atan2(last_gps[0]-lop_gps[0], last_gps[1]-lop_gps[1])
        print("new total_gyro", total_gyro)
    if dir_tick > 5:
        process_gameinformation()

        process_gyro()

        if tick_count % 5 == 0:
            point_cloud = np.array(lidar.getLayerPointCloud(2)[0:512])

        if tick_count == 10 or tick_count % 10 == 0 or (on_area_4 and tick_count % 10 == 0):
            if(speeds[0] == speeds[1]):
                process_lidar()

        if (tick_count % 5 == 0 and (len(action_list) == 0 or (isinstance(action_list[next_action], Acao) and action_list[next_action].name != "collect"))):
            process_camera(camera_plus.getImage(), "plus")
            process_camera(camera_sub.getImage(), "sub")
            process_camera(camera_front.getImage(), "front")

        max_velocity = 6.28
        turn_velocity = 6.28


def process_gameinformation():
    global score
    global remaining_time
    global next_action
    global tick_count

    if len(action_list) > 0 and isinstance(action_list[next_action], Acao):
        if map.coords_to_tile(0, 0) != map.coords_to_tile(last_gps[0], last_gps[1]):
            if tick_count % 5 == 0:
                # message = 'G' for game information
                message = struct.pack('c', 'G'.encode(encoding="utf-8", errors="ignore"))
                emitter.send(message)  # send message

            if receiver.getQueueLength() > 0 and tick_count % 5 == 1:
                receivedData = receiver.getBytes()
                rDataLen = len(receivedData)
                if rDataLen == 12:
                    tup = struct.unpack('c f i', receivedData)
                    if tup[0].decode("utf-8") == 'G':
                        score = tup[1]
                        remaining_time = tup[2]
                receiver.nextPacket()


def process_lidar():
    global total_gyro
    for i in range(len(point_cloud)):
        point = point_cloud[i]

        angle = total_gyro + math.pi/2

        coordX = front_gps[0] + math.cos(angle) * (point.x) - math.sin(angle) * (point.y)
        coordY = front_gps[1] + math.sin(angle) * (-point.x) - math.cos(angle) * (point.y)

        if on_area_4:
            p = 2
        else:
            p = 4

        coordX_d = front_gps[0] + math.cos(angle)*(point_cloud[(i+p) % 512].x) - math.sin(angle) * (point_cloud[(i+p) % 512].y)
        coordY_d = front_gps[1] + math.sin(angle)*(-point_cloud[(i+p) % 512].x) - math.cos(angle) * (point_cloud[(i+p) % 512].y)
        coordX_e = front_gps[0] + math.cos(angle)*(point_cloud[(i-p+512) % 512].x) - math.sin(angle) * (point_cloud[(i-p+512) % 512].y)
        coordY_e = front_gps[1] + math.sin(angle)*(-point_cloud[(i-p+512) % 512].x) - math.cos(angle) * (point_cloud[(i-p+512) % 512].y)

        ang = math.atan2((coordY_e-coordY_d), (coordX_d - coordX_e)) 

        dist_ant = dist_coords(coordX, coordY, coordX_d, coordY_d)
        dist_prox = dist_coords(coordX, coordY, coordX_e, coordY_e)
        dist_ap = dist_coords(coordX_e, coordY_e, coordX_d, coordY_d)

        if ang != math.pi and ang != 2*math.pi and ang != 0 and ang != -math.pi and ang != -2*math.pi:
            coordX = coordX - math.sin(ang)*0.005
        if ang != 3*math.pi/2 and ang != -math.pi/2 and ang != math.pi/2 and ang != -3*math.pi/2:
            coordY = coordY - math.cos(ang)*0.005

        if (dist_ap < 0.04) and (dist_ant < 0.01*p/2) and (dist_prox < 0.01*p/2):
            if on_area_4:
                map4.add_wall(coordX, coordY)
            map.add_wall(coordX, coordY)

    if on_area_4:
        map4.cleanup_matrix()
        map4.print_map()
    else:
        map.cleanup_matrix()
    map.print_map()


def is_wall(color):
    yes = False

    if abs(color[1]-2*color[0]) < 10 and abs(color[2]-color[1]-(color[1]-color[0])/6) < 3:
        yes = True

    if (color[0] == color[1] and color[1] == color[2]):
        yes = False
    if (color[2]-color[1] > abs(color[1]-color[0])):
        yes = False

    return yes


def not_ground(color):
    yes = True

    if abs(color[0] - 196) <= 5 and abs(color[1] - 196) <= 5 and abs(color[2] - 196) <= 5:
        yes = False
    if (abs(color[0] - 31) <= 5 and abs(color[1] - 31) <= 5 and abs(color[2] - 31) <= 5) or (abs(color[0] - 24) <= 5 and abs(color[1] - 24) <= 5 and abs(color[2] - 24) <= 5) or (abs(color[0] - 11) <= 5 and abs(color[2] - 11) <= 5 and abs(color[2] - 11) <= 5):
        yes = False
    if abs(color[0] - 170) <= 5 and abs(color[1] - 137) <= 5 and abs(color[2] - 76) <= 5:
        yes = False
    if abs(color[0] - 47) <= 5 and abs(color[1] - 47) <= 5 and abs(color[2] - 235) <= 5:
        yes = False
    if abs(color[0] - 110) <= 5 and abs(color[1] - 47) <= 5 and abs(color[2] - 189) <= 5:
        yes = False
    if abs(color[0] - 235) <= 5 and abs(color[1] - 47) <= 5 and abs(color[2] - 47) <= 5:
        yes = False
    if abs(color[0] - 25) <= 5 and abs(color[1] - 225) <= 5 and abs(color[2] - 25) <= 5:
        yes = False

    return yes


def find_victim(img):
    topwall = np.zeros(64, dtype=int)
    deltas = []
    x_right = -1
    x_left = 65

    for y in range(2, 31):
        for x in range(0, 64):
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


def find_extreme_points(M):
    topPixel = -1
    bottomPixel = -1
    leftPixel = -1
    rightPixel = -1

    for i in range(0, M.shape[0]):
        if sum(M[i][:]) > 0:
            #print("Topo: ",i)
            topPixel = i
            break

    for i in range(0, M.shape[1]):
        if sum(M[:, i]) > 0:
            #print("Esquerda: ",i)
            leftPixel = i
            break

    for i in reversed(range(0, M.shape[0])):
        if sum(M[i][:]) > 0:
            #print("Base: ",i)
            bottomPixel = i
            break

    for i in reversed(range(0, M.shape[1])):
        if sum(M[:, i]) > 0:
            #print("Direita: ",i)
            rightPixel = i
            break

    return topPixel, bottomPixel, leftPixel, rightPixel


def dfs_border(M, x, y):
    M[x, y] = 255
    if x < M.shape[0]-1 and not M[x+1][y]:
        dfs_border(M, x+1, y)
    if x > 0 and not M[x-1][y]:
        dfs_border(M, x-1, y)
    if y < M.shape[1]-1 and not M[x][y+1]:
        dfs_border(M, x, y+1)
    if y > 0 and not M[x][y-1]:
        dfs_border(M, x, y-1)

    return M


def identify_victim(img_data):
    img = np.array(np.frombuffer(img_data, np.uint8).reshape((camera_front.getHeight(), camera_front.getWidth(), 4)))

    imgcamera_hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    lower_white = np.array([0, 0, 50])
    upper_white = np.array([180, 100, 255])

    lower_black = np.array([0, 0, 0])
    upper_black = np.array([180, 255, 10])

    lower_red = np.array([150, 100, 100])
    upper_red = np.array([180, 255, 255])

    lower_yellow = np.array([10, 100, 100])
    upper_yellow = np.array([50, 255, 255])

    mask_white = cv2.inRange(imgcamera_hsv, lower_white, upper_white)
    mask_black = cv2.inRange(imgcamera_hsv, lower_black, upper_black)
    mask_red = cv2.inRange(imgcamera_hsv, lower_red, upper_red)
    mask_yellow = cv2.inRange(imgcamera_hsv, lower_yellow, upper_yellow)

    white = np.sum(mask_white == 255)
    black = np.sum(mask_black == 255)
    red = np.sum(mask_red == 255)
    yellow = np.sum(mask_yellow == 255)

    if collect_print:
        print(white, black, red, yellow)

    poster_lo = np.array([0, 0, 200])
    poster_hi = np.array([10, 10, 215])

    mask = cv2.inRange(imgcamera_hsv, poster_lo, poster_hi)
    imgcamera_hsv[mask > 0] = (255, 255, 255)
    cv2.imwrite('1.png', imgcamera_hsv)

    imgcamera_gray = cv2.cvtColor(imgcamera_hsv, cv2.COLOR_HSV2BGR)
    imgcamera_gray = cv2.cvtColor(imgcamera_gray, cv2.COLOR_BGR2GRAY)
    mask = cv2.inRange(imgcamera_gray, 164, 164)
    imgcamera_gray[mask > 0] = 255
    cv2.imwrite('2.png', imgcamera_gray)

    imgcamera_thresh = cv2.threshold(imgcamera_gray, 200, 255, cv2.THRESH_BINARY)[1]
    cv2.imwrite('3.png', imgcamera_thresh)
    topPixel, bottomPixel, leftPixel, rightPixel = find_extreme_points(imgcamera_thresh)
    imgcutted_Poster = imgcamera_thresh[topPixel:bottomPixel+1, leftPixel:rightPixel+1]

    #if leftPixel != -1:
     #   cv2.imwrite('cutted_Poster.png', imgcutted_Poster)

    branco = np.sum(imgcutted_Poster == 255)
    print("branco", branco)

    aux = 2*math.pi/512
    ae = front_gps[0] + dist_lidar_2(510) * (math.sin(total_gyro+2*aux))
    be = front_gps[1] + dist_lidar_2(510) * (math.cos(total_gyro+2*aux))
    ad = front_gps[0] + dist_lidar_2(2) * (math.sin(total_gyro-2*aux))
    bd = front_gps[1] + dist_lidar_2(2) * (math.cos(total_gyro-2*aux))

    ang_min = math.atan2(ad-ae, bd-be)
    ang_max = math.atan2(ae-ad, be-bd)

    if (branco > 150 or (branco > 100 and ((abs(total_gyro-ang_min) < 0.85 or 2*math.pi-abs(total_gyro-ang_min) < 0.85) or (abs(total_gyro-ang_max) < 0.85 or 2*math.pi-abs(total_gyro-ang_max) < 0.85)))) and bottomPixel-topPixel > 5 and rightPixel-leftPixel > 5 and (imgcutted_Poster[3, 1] or imgcutted_Poster[3, (rightPixel-leftPixel-1)]):
        M = np.zeros((imgcutted_Poster.shape[0]+3, imgcutted_Poster.shape[1]+2))
        M[1:imgcutted_Poster.shape[0]+2, 1:imgcutted_Poster.shape[1]+1] = 255
        cv2.imwrite('M1.png', M)
        M[2:imgcutted_Poster.shape[0]+2, 1:imgcutted_Poster.shape[1]+1] = imgcutted_Poster
        cv2.imwrite('M2.png', M)
        imgcutted_Letter = dfs_border(M, 0, 0) # pinta o preto em volta da vítima de branco
        imgcutted_Letter = imgcutted_Letter[1:imgcutted_Letter.shape[0]+1, 1:imgcutted_Letter.shape[1]+1]
        imgcutted_Letter = 255-imgcutted_Letter # inverte preto e branco
        s = 'cutted_Letter.png'
        cv2.imwrite(s, imgcutted_Letter)
        topPixel, bottomPixel, leftPixel, rightPixel = find_extreme_points(imgcutted_Letter) # acha extremos da letra branca

        if black > 0 or ((abs(total_gyro-ang_min) < 1 or 2*math.pi-abs(total_gyro-ang_min) < 1) or (abs(total_gyro-ang_max) < 1 or 2*math.pi-abs(total_gyro-ang_max) < 1)):
            if imgcutted_Letter[topPixel+1][int((leftPixel+rightPixel)//2)] == 255:
                return 'S'
            elif imgcutted_Letter[bottomPixel-1][int((leftPixel+rightPixel)//2)] == 255:
                return 'U'
            elif imgcutted_Letter[int((topPixel+bottomPixel)//2)-1][int((leftPixel+rightPixel)//2)] == 255:
                return 'H'
    else:
        
        if yellow > 5:
            return 'O'
        elif red > 5:
            return 'F'
        elif black > 25 or (black > 0 and ((abs(total_gyro-ang_min) < 1 or 2*math.pi-abs(total_gyro-ang_min) < 1) or (abs(total_gyro-ang_max) < 1 or 2*math.pi-abs(total_gyro-ang_max) < 1))):
            return 'C'
        elif white > 0:
            return 'P'

    return 'N'


def process_gyro():
    global last_gyro
    global total_gyro
    last_gyro = gyro.getValues()[1]
    total_gyro = total_gyro + last_gyro*time_step*0.001
    if total_gyro > math.pi:
        total_gyro -= 2*math.pi
    if total_gyro < -math.pi:
        total_gyro += 2*math.pi


def connection4_tile(x, y):
    if (map.matrix[y-1][x-1] == 8 or map.matrix[y-1][x-1] == 9 or map.matrix[y+1][x+1] == 8 or map.matrix[y+1][x+1] == 9 or map.matrix[y+1][x-1] == 8 or map.matrix[y-1][x+1] == 9 or map.matrix[y-1][x+1] == 8 or map.matrix[y+1][x-1] == 9):
        return True
    else:
        return False


def ground_color(colour_hsv, colour_rgb):
    if (abs(colour_rgb[0] - 31) <= 5 and abs(colour_rgb[1] - 31) <= 5 and abs(colour_rgb[2] - 31) <= 5) or (abs(colour_rgb[0] - 24) <= 5 and abs(colour_rgb[1] - 24) <= 5 and abs(colour_rgb[2] - 24) <= 5) or (abs(colour_rgb[0] - 11) <= 5 and abs(colour_rgb[1] - 11) <= 5 and abs(colour_rgb[2] - 11) <= 5):
        return 2
        # buraco
    elif is_wall(colour_rgb):
        return 1
        # parede
    elif abs(colour_hsv[0] - 0) <= 3 and abs(colour_hsv[1] - 0) <= 30:
        return 0
        # chão normal
    elif abs(colour_hsv[0] - 19) <= 3 and abs(colour_hsv[1] - 125) <= 30:
        return 3
        # pantano
    elif abs(colour_hsv[0] - 120) <= 3 and abs(colour_hsv[1] - 190) <= 30:
        return 6
        # blue (1-2)
    elif abs(colour_hsv[0] - 133) <= 3 and abs(colour_hsv[1] - 170) <= 30:
        return 7
        # purple (2-3)
    elif abs(colour_hsv[0] - 0) <= 3 and abs(colour_hsv[1] - 190) <= 30:
        return 8
        # red (3-4)
    elif abs(colour_hsv[0] - 60) <= 3 and abs(colour_hsv[1] - 220) <= 30:
        return 9
        #green (1-4)
    else:
        return 4
        # checkpoint


def process_colour():
    global next_ground

    img_data = camera_front.getImage()
    colour_img = np.array(np.frombuffer(img_data, np.uint8).reshape((camera_front.getHeight(), camera_front.getWidth(), 4)))
    colour_img_hsv = cv2.cvtColor(colour_img, cv2.COLOR_BGR2HSV)

    img_data_plus = camera_plus.getImage()
    colour_img_plus = np.array(np.frombuffer(img_data_plus, np.uint8).reshape((camera_front.getHeight(), camera_front.getWidth(), 4)))
    colour_img_plus_hsv = cv2.cvtColor(colour_img_plus, cv2.COLOR_BGR2HSV)

    img_data_sub = camera_sub.getImage()
    colour_img_sub = np.array(np.frombuffer(img_data_sub, np.uint8).reshape((camera_front.getHeight(), camera_front.getWidth(), 4)))
    colour_img_sub_hsv = cv2.cvtColor(colour_img_sub, cv2.COLOR_BGR2HSV)


    left_pixel = 39

    HSVleft = [colour_img_plus_hsv.item(39, left_pixel, 0), colour_img_plus_hsv.item(39, left_pixel, 1), colour_img_plus_hsv.item(39, left_pixel, 2)]
    RGBleft = [colour_img_plus.item(39, left_pixel, 2), colour_img_plus.item(39, left_pixel, 1), colour_img_plus.item(39, left_pixel, 0)]
    next_ground = ground_color(HSVleft, RGBleft)

    if navegation_print:
        print("next_ground_left:", next_ground)

    img_angle = math.atan((-(left_pixel)+31) * math.tan(0.84/2) / 32) + 1.03

    a = front_gps[0] + 0.0853 * (math.sin(total_gyro+img_angle))
    b = front_gps[1] + 0.0853 * (math.cos(total_gyro+img_angle))
    x, y = map.coords_to_tile_center(a, b)
    x2, y2 = map.coords_to_even_tile(a, b)
    c, d = map.tile_to_coords(x2, y2)

    raio = round((math.pi-(img_angle))*256/math.pi)
    raio = (raio+256) % 512

    if next_ground != 0 and next_ground != 1 and (x2 % 4 == 0 or abs(a-c) > 0.0045) and (y2 % 4 == 0 or abs(b-d) > 0.0045):
        if next_ground == 2 and dist_lidar_2(raio) > 0.0855 and dist_lidar_2((raio-2+512)%512) > 0.0855 and dist_lidar_2((raio+2)%512) > 0.0855 and (abs(map.matrix[y, x]) != 1 and abs(map.matrix[y-1, x]) != 1 and abs(map.matrix[y, x+1]) != 1 and abs(map.matrix[y, x-1]) != 1 and abs(map.matrix[y+1, x]) != 1) and (map.matrix[y, x] != 10 and map.matrix[y-1, x] != 10 and map.matrix[y, x+1] != 10 and map.matrix[y, x-1] != 10 and map.matrix[y+1, x] != 10):
            map.matrix[y-1][x-1] = next_ground
            map.matrix[y-1][x+1] = next_ground
            map.matrix[y+1][x-1] = next_ground
            map.matrix[y+1][x+1] = next_ground
            print(x, y)
            print("colocou ground", a, c, b, d)
        elif next_ground != 2:
            map.matrix[y-1][x-1] = next_ground
            map.matrix[y-1][x+1] = next_ground
            map.matrix[y+1][x-1] = next_ground
            map.matrix[y+1][x+1] = next_ground
            print(x, y)
            print("colocou ground", a, c, b, d)


    right_pixel = 23

    HSVright = [colour_img_sub_hsv.item(39, right_pixel, 0), colour_img_sub_hsv.item(39, right_pixel, 1), colour_img_sub_hsv.item(39, right_pixel, 2)]
    RGBright = [colour_img_sub.item(39, right_pixel, 2), colour_img_sub.item(39, right_pixel, 1), colour_img_sub.item(39, right_pixel, 0)]
    next_ground = ground_color(HSVright, RGBright)

    if navegation_print:
        print("next_ground_right:", next_ground)

    img_angle = math.atan((-(right_pixel)+31) * math.tan(0.84/2) / 32) - 1.03

    a = front_gps[0] + 0.0853 * (math.sin(total_gyro+img_angle))
    b = front_gps[1] + 0.0853 * (math.cos(total_gyro+img_angle))
    x, y = map.coords_to_tile_center(a, b)
    x2, y2 = map.coords_to_even_tile(a, b)
    c, d = map.tile_to_coords(x2, y2)

    raio = round((math.pi-(img_angle))*256/math.pi)
    raio = (raio+256) % 512

    if next_ground != 0 and next_ground != 1 and (x2 % 4 == 0 or abs(a-c) > 0.0045) and (y2 % 4 == 0 or abs(b-d) > 0.0045):
        if next_ground == 2 and dist_lidar_2(raio) > 0.0855 and dist_lidar_2((raio-2+512)%512) > 0.0855 and dist_lidar_2((raio+2)%512) > 0.0855 and (abs(map.matrix[y, x]) != 1 and abs(map.matrix[y-1, x]) != 1 and abs(map.matrix[y, x+1]) != 1 and abs(map.matrix[y, x-1]) != 1 and abs(map.matrix[y+1, x]) != 1) and (map.matrix[y, x] != 10 and map.matrix[y-1, x] != 10 and map.matrix[y, x+1] != 10 and map.matrix[y, x-1] != 10 and map.matrix[y+1, x] != 10):
            map.matrix[y-1][x-1] = next_ground
            map.matrix[y-1][x+1] = next_ground
            map.matrix[y+1][x-1] = next_ground
            map.matrix[y+1][x+1] = next_ground
            print(x, y)
            print("colocou ground", a, c, b, d)
        elif next_ground != 2:
            map.matrix[y-1][x-1] = next_ground
            map.matrix[y-1][x+1] = next_ground
            map.matrix[y+1][x-1] = next_ground
            map.matrix[y+1][x+1] = next_ground
            print(x, y)
            print("colocou ground", a, c, b, d)
        

    HSVfront = [colour_img_hsv.item(39, 31, 0), colour_img_hsv.item(39, 31, 1), colour_img_hsv.item(39, 31, 2)]
    RGBfront = [colour_img.item(39, 31, 2), colour_img.item(39, 31, 1), colour_img.item(39, 31, 0)]
    next_ground = ground_color(HSVfront, RGBfront)

    if navegation_print:
        print("next_ground_front:", next_ground)

    a = front_gps[0] + 0.0847 * (math.sin(total_gyro))
    b = front_gps[1] + 0.0847 * (math.cos(total_gyro))
    x, y = map.coords_to_tile_center(a, b)
    x2, y2 = map.coords_to_even_tile(a, b)
    c, d = map.tile_to_coords(x2, y2)

    if next_ground != 0 and next_ground != 1 and (x2 % 4 == 0 or abs(a-c) > 0.003) and (y2 % 4 == 0 or abs(b-d) > 0.003):
        if next_ground == 2 and dist_lidar_2(0) > 0.0846 and dist_lidar_2(5) > 0.0846 and dist_lidar_2(507) > 0.0846 and (abs(map.matrix[y, x]) != 1 and abs(map.matrix[y-1, x]) != 1 and abs(map.matrix[y, x+1]) != 1 and abs(map.matrix[y, x-1]) != 1 and abs(map.matrix[y+1, x]) != 1) and (map.matrix[y, x] != 10 and map.matrix[y-1, x] != 10 and map.matrix[y, x+1] != 10 and map.matrix[y, x-1] != 10 and map.matrix[y+1, x] != 10):
            map.matrix[y-1][x-1] = next_ground
            map.matrix[y-1][x+1] = next_ground
            map.matrix[y+1][x-1] = next_ground
            map.matrix[y+1][x+1] = next_ground
            print(x, y)
            print("colocou ground", a, c, b, d)
        elif next_ground == 2:
            next_ground = 0
        else:
            map.matrix[y-1][x-1] = next_ground
            map.matrix[y-1][x+1] = next_ground
            map.matrix[y+1][x-1] = next_ground
            map.matrix[y+1][x+1] = next_ground
            print(x, y)
            print("colocou ground", a, c, b, d)   
    else:
        next_ground = 0


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


def dist_coords(a, b, x, y):
    dist = ((a-x)**2 + (b-y)**2)**0.5
    return(dist)


# -------------------------------------------------------------------------------
# Funções de movimento


def call_lop():
    global call_tick

    if call_tick == 0:
        call_tick = tick_count
        
    print("chamando lop", (tick_count - call_tick) * 16)
    
    if (tick_count - call_tick) * 16 > 100:
        message = struct.pack('c', 'L'.encode())
        emitter.send(message)
        call_tick = 0


def update_vh(gyro):
    global v
    global h

    # print(gyro)
    if(abs(gyro-0) < 0.4):
        #print("para 0")
        h = [+0, +0, +4, +2, -4, -2, +4, +2, -4, -2, +0, +0, +4, +2, -4, -2]
        v = [+4, +2, +0, +0, +0, +0, +4, +2, +4, +2, -4, -2, -4, -2, -4, -2]

    elif(abs(gyro-math.pi/2) < 0.4):
        #print("para -math.pi/2")
        h = [+4, +2, +0, +0, +0, +0, +4, +2, +4, +2, -4, -2, -4, -2, -4, -2]
        v = [+0, +0, +4, +2, -4, -2, +4, +2, -4, -2, +0, +0, +4, +2, -4, -2]

    elif(abs(gyro+math.pi/2) < 0.4):
        #print("para +math.pi/2")
        h = [-4, -2, +0, +0, +0, +0, -4, -2, -4, -2, +4, +2, +4, +2, +4, +2]
        v = [+0, +0, +4, +2, -4, -2, +4, +2, -4, -2, +0, +0, +4, +2, -4, -2]

    elif(abs(gyro-math.pi) < 0.4) or (abs(gyro+math.pi) < 0.4):
        #print("para -math.pi ou +math.pi")
        h = [+0, +0, +4, +2, -4, -2, +4, +2, -4, -2, +0, +0, +4, +2, -4, -2]
        v = [-4, -2, +0, +0, +0, +0, -4, -2, -4, -2, +4, +2, +4, +2, +4, +2]

    elif(abs(gyro-math.pi/4) < 0.4):
        #print("para -math.pi/4")
        h = [+4, +2, +0, +0, +4, +2, +0, +0, -4, -2, +4, +2, -4, -2, -4, -2]
        v = [+4, +2, +4, +2, +0, +0, -4, -2, +0, +0, -4, -2, +4, +2, -4, -2]

    elif(abs(gyro-3*math.pi/4) < 0.4):
        #print("para -3*math.pi/4")
        h = [+4, +2, +0, +0, +4, +2, +0, +0, -4, -2, +4, +2, -4, -2, -4, -2]
        v = [-4, -2, -4, -2, +0, +0, +4, +2, +0, +0, +4, +2, -4, -2, +4, +2]

    elif(abs(gyro+math.pi/4) < 0.4):
        #print("para +math.pi/4")
        h = [-4, -2, +0, +0, -4, -2, +0, +0, +4, +2, -4, -2, +4, +2, +4, +2]
        v = [+4, +2, +4, +2, +0, +0, -4, -2, +0, +0, -4, -2, +4, +2, -4, -2] 

    elif(abs(gyro+3*math.pi/4) < 0.4):
        #print("para +3*math.pi/4")
        h = [-4, -2, +0, +0, -4, -2, +0, +0, +4, +2, -4, -2, +4, +2, +4, +2]
        v = [-4, -2, -4, -2, +0, +0, +4, +2, +0, +0, +4, +2, -4, -2, +4, +2] 

    else:
        #print("para nada")
        h = [+0, +0, +4, +2, -4, -2, +4, +2, -4, -2, +0, +0, +4, +2, -4, -2]
        v = [+4, +2, +0, +0, +0, +0, +4, +2, +4, +2, -4, -2, -4, -2, -4, -2]


def update_vh_4(gyro):
    global v_4
    global h_4

    # print(gyro)
    if(abs(gyro-0) < 0.4):
        #print("para 0")
        h_4 = [+0, +0, +0, +12, +6, +3, -12, -6, -3, +12, +6, +3, -12, -6, -3, +12, +6, +3, -12, -6, -3, +0, +0, +0]
        v_4 = [+12, +6, +3, +0, +0, +0, +0, +0, +0, +12, +6, +3, +12, +6, +3, -12, -6, -3, -12, -6, -3, -12, -6, -3]
    elif(abs(gyro-math.pi/2) < 0.4):
        #print("para -math.pi/2")
        h_4 = [+12, +6, +3, +0, +0, +0, +0, +0, +0, +12, +6, +3, +12, +6, +3, -12, -6, -3, -12, -6, -3, -12, -6, -3]
        v_4 = [+0, +0, +0, +12, +6, +3, -12, -6, -3, +12, +6, +3, -12, -6, -3, +12, +6, +3, -12, -6, -3, +0, +0, +0]
    elif(abs(gyro+math.pi/2) < 0.4):
        #print("para +math.pi/2")
        h_4 = [-12, -6, -3, +0, +0, +0, +0, +0, +0, -12, -6, -3, -12, -6, -3, +12, +6, +3, +12, +6, +3, +12, +6, +3]
        v_4 = [+0, +0, +0, +12, +6, +3, -12, -6, -3, +12, +6, +3, -12, -6, -3, +12, +6, +3, -12, -6, -3, +0, +0, +0]
    elif(abs(gyro-math.pi) < 0.4) or (abs(gyro+math.pi) < 0.4):
        #print("para -math.pi ou +math.pi")
        h_4 = [+0, +0, +0, +12, +6, +3, -12, -6, -3, +12, +6, +3, -12, -6, -3, +12, +6, +3, -12, -6, -3, +0, +0, +0]
        v_4 = [-12, -6, -3, +0, +0, +0, +0, +0, +0, -12, -6, -3, -12, -6, -3, +12, +6, +3, +12, +6, +3, +12, +6, +3]
    elif(abs(gyro-math.pi/4) < 0.4):
        h_4 = [+12, +6, +3, +0, +0, +0, +12, +6, +3, +0, +0, +0, -12, -6, -3, +12, +6, +3, -12, -6, -3, -12, -6, -3]
        v_4 = [+12, +6, +3, +12, +6, +3, +0, +0, +0, -12, -6, -3, +0, +0, +0, -12, -6, -3, +12, +6, +3, -12, -6, -3]
    elif(abs(gyro-3*math.pi/4) < 0.4):
        #print("para -3*math.pi/4")
        h_4 = [+12, +6, +3, +0, +0, +0, +12, +6, +3, +0, +0, +0, -12, -6, -3, +12, +6, +3, -12, -6, -3, -12, -6, -3]
        v_4 = [-12, -6, -3, -12, -6, -3, +0, +0, +0, +12, +6, +3, +0, +0, +0, +12, +6, +3, -12, -6, -3, +12, +6, +3]
    elif(abs(gyro+math.pi/4) < 0.4):
        #print("para +math.pi/4")
        h_4 = [-12, -6, -3, -12, -6, -3, +0, +0, +0, +12, +6, +3, +0, +0, +0, +12, +6, +3, -12, -6, -3, +12, +6, +3]
        v_4 = [+12, +6, +3, +0, +0, +0, +12, +6, +3, +0, +0, +0, -12, -6, -3, +12, +6, +3, -12, -6, -3, -12, -6, -3]
    elif(abs(gyro+3*math.pi/4) < 0.4):
        #print("para +3*math.pi/4")
        h_4 = [-12, -6, -3, +0, +0, +0, -12, -6, -3, +0, +0, +0, +12, +6, +3, -12, -6, -3, +12, +6, +3, +12, +6, +3]
        v_4 = [-12, -6, -3, -12, -6, -3, +0, +0, +0, +12, +6, +3, +0, +0, +0, +12, +6, +3, -12, -6, -3, +12, +6, +3]
    else:
        h_4 = [+0, +0, +0, +12, +6, +3, -12, -6, -3, +12, +6, +3, -12, -6, -3, +12, +6, +3, -12, -6, -3, +0, +0, +0]
        v_4 = [+12, +6, +3, +12, +6, +3, +12, +6, +3, +0, +0, +0, +0, +0, +0, -12, -6, -3, -12, -6, -3, -12, -6, -3]


def verify_wall(x, y, X, Y):
    if not (X > 0 and Y > 0 and X < max_map and Y < max_map and x > 0 and y > 0 and x < max_map and y < max_map):
        return False
    if not (map.matrix[Y][X] == 0 and abs(map.matrix[Y+1][X]) != 1 and abs(map.matrix[Y-1][X]) != 1 and abs(map.matrix[Y][X+1]) != 1 and abs(map.matrix[Y][X-1]) != 1):
        return False
    if not (abs(map.matrix[Y-1][X-1]) != 1 and abs(map.matrix[Y+1][X-1]) != 1 and abs(map.matrix[Y-1][X+1]) != 1 and abs(map.matrix[Y+1][X+1]) != 1):
        return False
    if not (abs(map.matrix[int((Y+y)/2)][int((X+x)/2)]) != 1 and (abs(map.matrix[int((Y+y)/2)+1][int((X+x)/2)]) != 1 or abs(map.matrix[int((Y+y)/2)-1][int((X+x)/2)]) != 1) and (abs(map.matrix[int((Y+y)/2)][int((X+x)/2)+1]) != 1 or abs(map.matrix[int((Y+y)/2)][int((X+x)/2)-1]) != 1)):
        return False
    if not ((abs(map.matrix[int((Y+y)/2)+1][int((X+x)/2)]) != 1 or abs(map.matrix[int((Y+y)/2)][int((X+x)/2)-1]) != 1) and (abs(map.matrix[int((Y+y)/2)-1][int((X+x)/2)]) != 1 or abs(map.matrix[int((Y+y)/2)][int((X+x)/2)+1]) != 1) and (abs(map.matrix[int((Y+y)/2)+1][int((X+x)/2)]) != 1 or abs(map.matrix[int((Y+y)/2)][int((X+x)/2)+1]) != 1) and (abs(map.matrix[int((Y+y)/2)-1][int((X+x)/2)]) != 1 or abs(map.matrix[int((Y+y)/2)][int((X+x)/2)-1]) != 1)):
        return False
    if not (map.matrix[int((Y+y)/2)-1][int((X+x)/2)-1] != 2 and map.matrix[int((Y+y)/2)+1][int((X+x)/2)+1] != 2 and map.matrix[int((Y+y)/2)-1][int((X+x)/2)+1] != 2 and map.matrix[int((Y+y)/2)+1][int((X+x)/2)-1] != 2):
        return False
    if not (map.matrix[Y+1][X] != 2 and map.matrix[Y-1][X] != 2 and map.matrix[Y][X+1] != 2 and map.matrix[Y][X-1] != 2):
        return False
    if not (map.matrix[Y-1][X+1] != 2 and map.matrix[Y+1][X-1] != 2 and map.matrix[Y-1][X-1] != 2 and map.matrix[Y+1][X+1] != 2):
        return False
    
    return True


def verify_wall_2(x, y, X, Y):
    if not (X > 0 and Y > 0 and X < max_map and Y < max_map and x > 0 and y > 0 and x < max_map and y < max_map):
        return False
    
    if not (map.matrix[Y][X] == 0 and abs(map.matrix[int((Y+y)/2)][int((X+x)/2)]) != 1):
        return False
    if not ((abs(map.matrix[int((Y+y)/2)+1][int((X+x)/2)]) != 1 or abs(map.matrix[int((Y+y)/2)-1][int((X+x)/2)]) != 1) and (abs(map.matrix[int((Y+y)/2)][int((X+x)/2)+1]) != 1 or abs(map.matrix[int((Y+y)/2)][int((X+x)/2)-1]) != 1)):
        return False
    if not ((abs(map.matrix[int((Y+y)/2)+1][int((X+x)/2)]) != 1 or abs(map.matrix[int((Y+y)/2)][int((X+x)/2)-1]) != 1) and (abs(map.matrix[int((Y+y)/2)-1][int((X+x)/2)]) != 1 or abs(map.matrix[int((Y+y)/2)][int((X+x)/2)+1]) != 1) and (abs(map.matrix[int((Y+y)/2)+1][int((X+x)/2)]) != 1 or abs(map.matrix[int((Y+y)/2)][int((X+x)/2)+1]) != 1) and (abs(map.matrix[int((Y+y)/2)-1][int((X+x)/2)]) != 1 or abs(map.matrix[int((Y+y)/2)][int((X+x)/2)-1]) != 1)):
        return False
    if not ((abs(map.matrix[Y-1, X]) != 1 or abs(map.matrix[Y, X-1]) != 1) and (abs(map.matrix[Y-1, X]) != 1 or abs(map.matrix[Y, X+1]) != 1) and (abs(map.matrix[Y+1, X]) != 1 or abs(map.matrix[Y, X-1]) != 1) and (abs(map.matrix[Y+1, X]) != 1 or abs(map.matrix[Y, X+1]) != 1)):
        return False

    if not (map.matrix[int((Y+y)/2)-1][int((X+x)/2)-1] != 2 and map.matrix[int((Y+y)/2)+1][int((X+x)/2)+1] != 2 and map.matrix[int((Y+y)/2)-1][int((X+x)/2)+1] != 2 and map.matrix[int((Y+y)/2)+1][int((X+x)/2)-1] != 2):
        return False
    if not (map.matrix[Y+1][X] != 2 and map.matrix[Y-1][X] != 2 and map.matrix[Y][X+1] != 2 and map.matrix[Y][X-1] != 2):
        return False
    if not (map.matrix[Y-1][X+1] != 2 and map.matrix[Y+1][X-1] != 2 and map.matrix[Y-1][X-1] != 2 and map.matrix[Y+1][X+1] != 2):
        return False

    return True


def verify_wall_4(x, y, X, Y):
    if X-x == 0:
        posx = 0
    elif X > x:
        posx = 1
    else:
        posx = -1

    if Y-y == 0:
        posy = 0
    elif Y > y:
        posy = 1
    else:
        posy = -1

    # paredes que o robo bate no começo
    if backtracking:
        for i in range(-5, +6):
            for j in range(-5, +6):
                wx, wy = map4.tile_to_coords(x-i, y-j)
                zx, zy = map.coords_to_tile(wx, wy)
                if i > -4 and i < 4 and j > -4 and j < 4 and abs(i) + abs(j) < 6:
                    if y-j > 400 or x-i > 400 or y-j < 0 or x-i < 0 or abs(map4.matrix[y-j, x-i]) == 1:
                        return False
                if map.matrix[zy, zx] == 2:
                    return False

    # paredes no caminho
    atualx, atualy = x, y
    while (abs(atualx - X) > 2 or abs(atualy - Y) > 2):
        for i in range(0, 11):
            auxx, auxy = atualx + posy * (2-i), atualy - posx * (2-i)
            auxx2, auxy2 = atualx + posy * (5-i), atualy - posx * (5-i)
            wx, wy = map4.tile_to_coords(auxx2, auxy2)
            zx, zy = map.coords_to_tile(wx, wy)

            if i < 5 and (auxx > 400 or auxx < 0 or auxy > 400 or auxy < 0 or abs(map4.matrix[auxy, auxx]) == 1):
                return False
            if map.matrix[zy, zx] == 2:
                return False
        atualx = atualx + posx
        atualy = atualy + posy

    # paredes que o robo bate ao chegar
    for i in range(-5, +6):
        for j in range(-5, +6):
            wx, wy = map4.tile_to_coords(X-i, Y-j)
            zx, zy = map.coords_to_tile(wx, wy)
            if i > -4 and i < 4 and j > -4 and j < 4 and abs(i) + abs(j) < 6:
                if Y-j > 400 or X-i > 400 or Y-j < 0 or X-i < 0 or abs(map4.matrix[Y-j, X-i]) == 1:
                    return False
            if map.matrix[zy, zx] == 2:
                return False

    return True


def can_go(X, Y):
    for i in range(-1, +2):
        for j in range(-1, +2):
            if Y-i > 400 or X-j > 400 or Y-i < 0 or X-j < 0 or abs(map.matrix[Y-j, X-i]) == 1:
                if navegation_print:
                    print("Cant go", X, Y, X-i, Y-j)
                return False

    return True


def can_go_4(X, Y):
    for i in range(-5, +6):
        for j in range(-5, +6):
            wx, wy = map4.tile_to_coords(X-i, Y-j)
            zx, zy = map.coords_to_tile(wx, wy)
            if i > -4 and i < 4 and j > -4 and j < 4 and abs(i) + abs(j) < 6:
                if Y-j > 400 or X-i > 400 or Y-j < 0 or X-i < 0 or abs(map4.matrix[Y-j, X-i]) == 1:
                    if navegation_print:
                        print("Cant go", X, Y, X-i, Y-j)
                    return False
            if map.matrix[zy, zx] == 2:
                if navegation_print:
                    print("Cant go buraco", X, Y, X-i, Y-j)
                return False

    return True


def bfs(Sx, Sy, Tx, Ty):
    # print("BFS")
    # print("***************************************")
    # print(Sx,Sy,Tx,Ty)

    for x in range(max_map):
        for y in range(max_map):
            dist[x, y] = 99999999
            parent[x, y, 0] = 0
            parent[x, y, 1] = 0
            parent2[x, y, 0] = 0
            parent2[x, y, 1] = 0

    dist[Sx, Sy] = 0

    q = deque()
    q.append((Sx, Sy, 1))

    fake_gyro = total_gyro

    while True:
        [x, y, cd] = q.popleft()
        #print(x, y)

        if cd > 1:
            q.append((x, y, cd-1))
        else:
            if [x, y] != [Sx, Sy]:
                [X, Y] = parent[x, y]
                #print([X, Y], "->", [x, y])
                fake_gyro = math.atan2((x-X), (y-Y))  
                update_vh(fake_gyro)
            #print([x, y], fake_gyro)

            for aux in range(0, 16):
                X = x+h[aux]
                Y = y+v[aux]

                # peso de distância (no eixo ou diagonal)
                if h[aux] == 0 or v[aux] == 0:
                    if abs(h[aux]) == 2 or abs(v[aux]) == 2:
                        peso = 5
                    else:
                        peso = 10
                else:
                    if abs(h[aux]) == 2:
                        peso = 7
                    else:
                        peso = 14

                # peso para virar
                peso = peso + int(math.floor((aux+1)/4))*2

                # peso pântano
                if Y >= map.yMin and Y <= map.yMax and X >= map.xMin and X <= map.xMax and (map.matrix[Y+1, X+1] == 3 or map.matrix[Y-1, X-1] == 3 or map.matrix[Y-1, X+1] == 3 or map.matrix[Y+1, X-1] == 3):
                    peso = peso * 2

                if Y >= map.yMin and Y <= map.yMax and X >= map.xMin and X <= map.xMax and ((x % 4 == 0 and y % 4 == 0) or aux % 2 == 1) and verify_wall(x, y, X, Y) and dist[X][Y] > dist[x][y]+peso:
                    q.append((X, Y, peso))
                    parent[X][Y] = [x, y]
                    dist[X][Y] = dist[x][y]+peso

                    if X == Tx and Y == Ty:
                        #print("montou caminho")
                        return True

            if len(q) == 0:
                if navegation_print:
                    print("não montou caminho")
                return False


def bfs_4(Sx, Sy, Tx, Ty):
    for x in range(max_map):
        for y in range(max_map):
            dist[x, y] = 99999999
            parent[x, y, 0] = 0
            parent[x, y, 1] = 0
            parent2[x, y, 0] = 0
            parent2[x, y, 1] = 0

    dist[Sx, Sy] = 0

    q = deque()
    q.append((Sx, Sy))

    fake_gyro = total_gyro

    cont = 0

    while True:
        cont = cont + 1
        [x, y] = q.popleft()

        if [x, y] != [Sx, Sy]:
            [X, Y] = parent[x, y]
            #print([X, Y], "->", [x, y])
            fake_gyro = math.atan2((x-X), (y-Y))   
            update_vh_4(fake_gyro)
        #print([x, y], fake_gyro)

        for aux in range(0, 24):
            X = x+h_4[aux]
            Y = y+v_4[aux]

            if X >= map4.xMin and X < map4.xMax and Y >= map4.yMin and Y < map4.yMax and verify_wall_4(x, y, X, Y) and dist[X][Y] > dist[x][y]+1:
                q.append((X, Y))
                parent[X][Y] = [x, y]
                dist[X][Y] = dist[x][y]+1
                if bfs_print:
                    print(X, Y)

                if abs(X - Tx) <= 2 and abs(Y - Ty) <= 2:
                    print("montou caminho")
                    parent[Tx][Ty] = [x, y]
                    dist[Tx][Ty] = dist[x][y]+1

                    return True

        if len(q) == 0:
            if navegation_print:
                print("não montou caminho")
            return False

        if cont > 500:
            if navegation_print:
                print("contou")
            return False


def invert_parent(a, b, c, d):
    cont = 0
    while([a, b] != [c, d]):
        cont = cont + 1
        if (cont > 100):
            if bfs_print:
                print("deu ruim no while no invert parent")
            break
        parent2[parent[a, b][0], parent[a, b][1]] = [a, b]
        #print(a, b, parent[a, b])
        [a, b] = parent[a, b]


def backtrack(actualX, actualY):
    global backtracking
    global on_area_4
    global emergencial_backwardX
    global emergencial_backwardY

    backtracking = True
    found = False
    back_wantedX = int((max_map-1)/2)
    back_wantedY = int((max_map-1)/2)

    if navegation_print:
        print("backtrack", actualX, actualY)

    while(not found):
        if(backwardX.empty() == False):
            x = backwardX.get()
            y = backwardY.get()
            backwardX_save.put(x)
            backwardY_save.put(y)
            x = int(x)
            y = int(y)
            emergencial_backwardX.append(x)
            emergencial_backwardY.append(y)
            print(x, y)
        else:
            break

        if [x, y] == last_4:
            back_wantedX = x
            back_wantedY = y
            found = True
            on_area_4 = True
            break

        for aux in range(0, 16):
            X = x+h[aux]
            Y = y+v[aux]

            if ((x % 4 == 0 and y % 4 == 0) or aux % 2 == 1) and not found and map.mark[Y][X] == 0 and verify_wall(x, y, X, Y):
                if navegation_print:
                    print("path achou", X, Y)
                back_wantedX = x
                back_wantedY = y

                found = True
                break

        if x == int((max_map-1)/2) and y == int((max_map-1)/2) and not found:
            back_wantedX = x
            back_wantedY = y
            #print("path não achou", x, y)
            found = True
            break

    if (actualX != back_wantedX or actualY != back_wantedY):
        if bfs(actualX, actualY, back_wantedX, back_wantedY):
            [x, y] = [actualX, actualY]
            invert_parent(back_wantedX, back_wantedY, x, y)
            # print("back")
            cont = 0
            while([x, y] != [back_wantedX, back_wantedY]):
                cont = cont + 1
                #print("action", parent2[x, y])
                action_list.append(("walk_to", map.tile_to_coords(parent2[x, y][0], parent2[x, y][1])[0], map.tile_to_coords(parent2[x, y][0], parent2[x, y][1])[1], map.tile_to_coords(x, y)[0], map.tile_to_coords(x, y)[1]))
                [x, y] = parent2[x, y]
        else:
            for i in range (len(emergencial_backwardX)):
                x, y = map.tile_to_coords(emergencial_backwardX[i], emergencial_backwardY[i])
                action_list.append(("walk_to", x, y, last_gps[0], last_gps[1]))
        emergencial_backwardX = []
        emergencial_backwardY = []
    

def backtrack_4(actualX, actualY):
    global backtracking
    global emergencial_backwardX
    global emergencial_backwardY

    backtracking = True
    found = False
    back_wantedX = int((max_map-1)/2)
    back_wantedY = int((max_map-1)/2)

    while(not found):
        if(backwardX_4.empty() == False):
            x = backwardX_4.get()
            y = backwardY_4.get()
            backwardX_save.put(x)
            backwardY_save.put(y)
            x = int(x)
            y = int(y)
            emergencial_backwardX.append(x)
            emergencial_backwardY.append(y)
            if navegation_print:
                print("backward puxando", x, y)
        else:
            if navegation_print:
                print("acabou back")
            break

        if abs(x-(max_map-1)/2) <= 2 and abs(y-(max_map-1)/2) <= 2:
            back_wantedX = x
            back_wantedY = y
            found = True
            break

        for aux in range(0, 24):
            X = x+h_4[aux]
            Y = y+v_4[aux]

            mwx, mwy = map4.tile_to_coords(x, y)
            mzx, mzy = map.coords_to_even_tile(mwx, mwy)
            wx, wy = map4.tile_to_coords(X, Y)
            zx, zy = map.coords_to_tile(wx, wy)
            zx2, zy2 = map.coords_to_even_tile(wx, wy)

            if not found and verify_wall_4(x, y, X, Y) and map4.mark[Y][X] == 0 and (aux % 3 == 2 or (zx % 4 == 0 and zy % 4 == 0) or (aux % 3 == 1 and not connection4_tile(mzx, mzy) and not connection4_tile(zx2, zy2))):
                if navegation_print:
                    print("path achou", x, y, X, Y)
                back_wantedX = x
                back_wantedY = y
                found = True
                break

    if (actualX != back_wantedX or actualY != back_wantedY):
        if navegation_print:
            print("bfs4")
        if bfs_4(actualX, actualY, back_wantedX, back_wantedY):
            [x, y] = [actualX, actualY]
            invert_parent(back_wantedX, back_wantedY, x, y)
            # print("back")
            cont = 0
            while([x, y] != [back_wantedX, back_wantedY]):
                cont = cont + 1
                if (cont > 500):
                    print("deu ruim depois da bfs 4")
                    break
                #print("action", parent2[x, y])
                action_list.append(("walk_to", map4.tile_to_coords(parent2[x, y][0], parent2[x, y][1])[0], map4.tile_to_coords(parent2[x, y][0], parent2[x, y][1])[1], map4.tile_to_coords(x, y)[0], map4.tile_to_coords(x, y)[1]))
                [x, y] = parent2[x, y]
                if bfs_print:
                    print(x, y)
        else:
            for i in range (len(emergencial_backwardX)):
                x, y = map4.tile_to_coords(emergencial_backwardX[i], emergencial_backwardY[i])
                action_list.append(("walk_to", x, y, last_gps[0], last_gps[1]))
        emergencial_backwardX = []
        emergencial_backwardY = []


def solve(x, y):
    global backtracking
    global on_area_4
    global map4
    global action_list

    backtracking = False

    while(not backwardX_save.empty()):
        backwardX_save.get()
        backwardY_save.get()

    map.mark[y, x] = -1
    update_vh(total_gyro)
    for aux in range(0, 16):
        X = x+h[aux]
        Y = y+v[aux]

        if ((x % 4 == 0 and y % 4 == 0) or aux % 2 == 1) and map.mark[Y][X] == 0 and verify_wall(x, y, X, Y):
            action_list.append(("walk_to", map.tile_to_coords(X, Y)[0], map.tile_to_coords(X, Y)[1], last_gps[0], last_gps[1]))
            if navegation_print:
                print("quero ir", X, Y)

            if connection4_tile(x, y):
                if finished_4 or x % 4 == 2 or y % 4 == 2 or not backwardX_4.empty():
                    action_list = []
                    a = backwardX.get() # area 4 por último
                    b = backwardY.get()
                    a = int(a)
                    b = int(b)

                    while(connection4_tile(a, b)):
                        a = backwardX.get() 
                        b = backwardY.get()
                        a = int(a)
                        b = int(b)
                    
                    backwardX.put(a)
                    backwardY.put(b)

                    backtrack(x, y)
                else:
                    map4 = Map4()
                    on_area_4 = True
                    #backwardX.put(x)
                    #backwardY.put(y)
                    backwardX_4.put(200)
                    backwardY_4.put(200)
                    if navegation_print:
                        print("entrei area 4")
            else:
                if abs(h[aux]) > 2 and abs(v[aux]) > 2:
                    for i in range(2):
                        backwardX.put(x+2*i*(h[aux]/abs(h[aux])))
                        backwardY.put(y+2*i*(v[aux]/abs(v[aux])))
                else:
                    backwardX.put(x)
                    backwardY.put(y)

            break

        if aux == 15:
            if navegation_print:
                print("backkkk")
            backtrack(x, y)


def solve_4(x, y):
    global backtracking
    global on_area_4
    global action_list
    global finished_4
    global last_4

    backtracking = False
    while(not backwardX_save.empty()):
        backwardX_save.get()
        backwardY_save.get()
        
    if navegation_print:
        print("4", x, y)

    map4.mark[y, x] = -1
    update_vh_4(total_gyro)

    mwx, mwy = map4.tile_to_coords(x, y)
    mzx, mzy = map.coords_to_even_tile(mwx, mwy)

    if abs(x-(max_map-1)/2) < 3 and abs(y-(max_map-1)/2) < 3:
        print("finaliza area 4")
        on_area_4 = False
        finished_4 = True
        map.mark[mzy, mzx] = -1
        backtrack(mzx, mzy)
    elif [mzx, mzy] == last_4:
        print("back to area 4")
        a = backwardX_4.get() 
        b = backwardY_4.get()
        a = int(a)
        b = int(b)

        wx, wy = map4.tile_to_coords(a, b)
        zx, zy = map.coords_to_even_tile(wx, wy)

        while([a, b] != [200, 200] and connection4_tile(zx, zy)):
            a = backwardX_4.get() 
            b = backwardY_4.get()
            a = int(a)
            b = int(b)

            wx, wy = map4.tile_to_coords(a, b)
            zx, zy = map.coords_to_even_tile(wx, wy)
        
        backwardX_4.put(a)
        backwardY_4.put(b)
        backtrack_4(x, y)
    else:
        for aux in range(0, 24):
            X = x+h_4[aux]
            Y = y+v_4[aux]

            if X % 3 == 1:
                X = X + 1
            elif X % 3 == 0:
                X = X - 1 
            if Y % 3 == 1:
                Y = Y + 1
            elif Y % 3 == 0:
                Y = Y - 1 

            wx, wy = map4.tile_to_coords(X, Y)
            zx, zy = map.coords_to_tile(wx, wy)
            zx2, zy2 = map.coords_to_even_tile(wx, wy)

            if verify_wall_4(x, y, X, Y) and map4.mark[Y][X] == 0 and (aux % 3 == 2 or (zx % 4 == 0 and zy % 4 == 0) or (aux % 3 == 1 and not connection4_tile(mzx, mzy) and not connection4_tile(zx2, zy2))):
                print(wx, wy, zx2, zy2)
                if navegation_print:
                    print("quero ir", X, Y)
                action_list.append(("walk_to", wx, wy, mwx, mwy))

                if mzx % 4 == 0 and mzy % 4 == 0 and map.mark[zy2, zx2] == 0 and connection4_tile(mzx, mzy) and not connection4_tile(zx2, zy2):
                    last_4 = [mzx, mzy]
                    on_area_4 = False
                    backwardX.put(mzx)
                    backwardY.put(mzy)
                    action_list = [("walk_to", map.tile_to_coords(zx2, zy2)[0], map.tile_to_coords(zx2, zy2)[1], last_gps[0], last_gps[1])]
                    print("saindo área 4 por enquanto")
                    
                elif map.mark[zy2, zx2] != 0 and connection4_tile(mzx, mzy):
                    action_list = []
                    a = backwardX_4.get() 
                    b = backwardY_4.get()
                    a = int(a)
                    b = int(b)

                    wx, wy = map4.tile_to_coords(a, b)
                    zx, zy = map.coords_to_even_tile(wx, wy)

                    while(connection4_tile(zx, zy)):
                        a = backwardX_4.get() 
                        b = backwardY_4.get()
                        a = int(a)
                        b = int(b)

                        wx, wy = map4.tile_to_coords(a, b)
                        zx, zy = map.coords_to_even_tile(wx, wy)
                    
                    backwardX_4.put(a)
                    backwardY_4.put(b)
                    backtrack_4(x, y)
                else:
                    if abs(h_4[aux]) > 3:
                        if abs(v_4[aux]) > 3:
                            for i in range(int(abs(h_4[aux])/3)):
                                backwardX_4.put(x+i*(h_4[aux]/abs(h_4[aux]))*3)
                                backwardY_4.put(y+i*(v_4[aux]/abs(v_4[aux]))*3)
                        else:
                            for i in range(int(abs(h_4[aux])/3)):
                                backwardX_4.put(x+i*(h_4[aux]/abs(h_4[aux]))*3)
                                backwardY_4.put(y)
                    else:
                        if abs(v_4[aux]) > 3:
                            for i in range(int(abs(v_4[aux])/3)):
                                backwardX_4.put(x)
                                backwardY_4.put(y+i*(v_4[aux]/abs(v_4[aux]))*3)
                        else:
                            backwardX_4.put(x)
                            backwardY_4.put(y)

                break

            if aux == 23:
                if navegation_print:
                    print("backkkk")
                backtrack_4(x, y)


def backspawn():
    global semtempo

    print("ACABOU O TEMPO")

    actualX, actualY = map.coords_to_even_tile(last_gps[0], last_gps[1])

    if on_area_4:
        print("na área 4")
        actualX4, actualY4 = map4.coords_to_tile(last_gps[0], last_gps[1])      

        if bfs_4(actualX4, actualY4, 200, 200):
            [x, y] = [actualX4, actualY4]
            invert_parent(200, 200, x, y)
            # print("back")
            cont = 0
            while([x, y] != [200, 200]):
                #print("action", parent2[x, y])
                action_list.append(("walk_to", map4.tile_to_coords(parent2[x, y][0], parent2[x, y][1])[0], map4.tile_to_coords(parent2[x, y][0], parent2[x, y][1])[1], map4.tile_to_coords(x, y)[0], map4.tile_to_coords(x, y)[1]))
                [x, y] = parent2[x, y]

        a, b = map4.tile_to_coords(200, 200)
        actualX, actualY = map.coords_to_even_tile(a, b)
    elif not backwardX_4.empty():
        print("tem q voltar para a área 4 antes")
        if bfs(actualX, actualY, last_4[0], last_4[1]):
            [x, y] = [actualX, actualY]
            invert_parent(last_4[0], last_4[1], x, y)
            # print("back")
            cont = 0
            while([x, y] != last_4):
                #print("action", parent2[x, y])
                action_list.append(("walk_to", map.tile_to_coords(parent2[x, y][0], parent2[x, y][1])[0], map.tile_to_coords(parent2[x, y][0], parent2[x, y][1])[1], map.tile_to_coords(x, y)[0], map.tile_to_coords(x, y)[1]))
                [x, y] = parent2[x, y]

        a, b = map.tile_to_coords(last_4[0], last_4[1])
        actualX4, actualY4 = map4.coords_to_tile(a, b)      

        if bfs_4(actualX4, actualY4, 200, 200):
            [x, y] = [actualX4, actualY4]
            invert_parent(200, 200, x, y)
            # print("back")
            cont = 0
            while([x, y] != [200, 200]):
                #print("action", parent2[x, y])
                action_list.append(("walk_to", map4.tile_to_coords(parent2[x, y][0], parent2[x, y][1])[0], map4.tile_to_coords(parent2[x, y][0], parent2[x, y][1])[1], map4.tile_to_coords(x, y)[0], map4.tile_to_coords(x, y)[1]))
                [x, y] = parent2[x, y]

        a, b = map4.tile_to_coords(200, 200)
        actualX, actualY = map.coords_to_even_tile(a, b)

    print("vai pro spawn")
    if bfs(actualX, actualY, 200, 200):
        [x, y] = [actualX, actualY]
        invert_parent(200, 200, x, y)
        # print("back")
        cont = 0
        while([x, y] != [200, 200]):
            #print("action", parent2[x, y])
            action_list.append(("walk_to", map.tile_to_coords(parent2[x, y][0], parent2[x, y][1])[0], map.tile_to_coords(parent2[x, y][0], parent2[x, y][1])[1], map.tile_to_coords(x, y)[0], map.tile_to_coords(x, y)[1]))
            [x, y] = parent2[x, y]

    semtempo = True


def find_action():
    global backtracking
    global temp_collect

    x, y = map.coords_to_even_tile(last_gps[0], last_gps[1])
    if navegation_print:
        print("=====================================")
        print("334:", x, y)

    if victim_print:   
        print("vitimas")
    cont = 0

    q = [0, 0]
    temp_collect = 2000
    victim_remove = -1

    if not semtempo:
        print(len(victim_list))
        for v in victim_list: 
            [a, b] = [v[1], v[2]]
            [ae, be] = v[6]
            [ad, bd] = v[7]

            vi = [ad-ae, bd-be]
            u = [-1*vi[1], vi[0]]

            w = [vi[0] * (0.04 / math.sqrt(vi[0]**2 + vi[1]**2)), vi[1] * (0.04 / math.sqrt(vi[0]**2 + vi[1]**2))]
            uw = [-1*w[1], w[0]]

            w2 = [-1*w[0], -1*w[1]]
            uw2 = [w2[1], -1*w2[0]]

            ponto_x = u[0] * (0.06 / math.sqrt(vi[0]**2 + vi[1]**2)) + a
            ponto_y = u[1] * (0.06 / math.sqrt(vi[0]**2 + vi[1]**2)) + b 

            ponto_x_2 = uw[0] * (0.06 / math.sqrt(w[0]**2 + w[1]**2)) + a + w[0]
            ponto_y_2 = uw[1] * (0.06 / math.sqrt(w[0]**2 + w[1]**2)) + b + w[1]

            ponto_x_3 = uw2[0] * (0.06 / math.sqrt(w2[0]**2 + w2[1]**2)) + a + w2[0]
            ponto_y_3 = uw2[1] * (0.06 / math.sqrt(w2[0]**2 + w2[1]**2)) + b + w2[1]

            vx,vy = map.coords_to_tile(v[1], v[2])

            walla, wallb = map.coords_to_even_tile(ponto_x, ponto_y)
            walla4, wallb4 = map4.coords_to_tile(ponto_x, ponto_y)
            walla_2, wallb_2 = map.coords_to_even_tile(ponto_x_2, ponto_y_2)
            walla4_2, wallb4_2 = map4.coords_to_tile(ponto_x_2, ponto_y_2)
            walla_3, wallb_3 = map.coords_to_even_tile(ponto_x_3, ponto_y_3)
            walla4_3, wallb4_3 = map4.coords_to_tile(ponto_x_3, ponto_y_3)

            actualX, actualY = map.coords_to_even_tile(last_gps[0], last_gps[1])
            actualX4, actualY4 = map4.coords_to_tile(last_gps[0], last_gps[1])

            if victim_print:   
                print(vx, vy, end='')

            if victim_print:   
                print(" e seria aqui", walla, wallb, map.mark[wallb, walla], map4.mark[wallb4, walla4], map.matrix[wallb, walla], map4.matrix[wallb4, walla4])
                print(dist_coords(last_gps[0], last_gps[1], v[1], v[2]))
            
            if (dist_coords(last_gps[0], last_gps[1], v[1], v[2]) < 0.075) or (dist_coords(last_gps[0], last_gps[1], v[1], v[2]) < 0.092 and ((not on_area_4 and not can_go(walla, wallb)) or (on_area_4 and not can_go_4(walla4, wallb4)))) or (dist_coords(last_gps[0], last_gps[1], v[1], v[2]) < 0.09 and not on_area_4 and (abs(v[3]-0) > 0.4 and abs(v[3]-math.pi) > 0.4 and abs(v[3]+math.pi) > 0.4 and abs(v[3]-math.pi/2) > 0.4 and abs(v[3]+math.pi/2) > 0.4)) or (dist_coords(last_gps[0], last_gps[1], v[1], v[2]) < 0.12 and ((not on_area_4 and not can_go(walla, wallb) and not can_go(walla_2, wallb_2) and not can_go(walla_3, wallb_3)) or (on_area_4 and not can_go_4(walla4, wallb4) and not can_go_4(walla4_2, wallb4_2) and not can_go_4(walla4_3, wallb4_3)))):
                ang = math.atan2(v[1] - last_gps[0], v[2] - last_gps[1])  
                ang_min, ang_max = v[3], v[4]
                if victim_print:   
                    print("vitima perto", round(ang, 3), round(v[3], 3), round(v[4], 3))
                    print(round(abs(ang-ang_min), 3), round(2*math.pi-abs(ang-ang_min), 3), round(abs(ang-ang_max), 3), round(2*math.pi-abs(ang-ang_max), 3))
                if ((ang_min >= 0 and (ang > ang_min or ang < ang_max)) or (ang_min < 0 and ang > ang_min and ang < ang_max)) and (abs(ang-ang_min) > 0.47 and 2*math.pi-abs(ang-ang_min) > 0.47) and (abs(ang-ang_max) > 0.47 and 2*math.pi-abs(ang-ang_max) > 0.47):
                    if q[0] != 0:
                        if dist_coords(a, b, q[0], q[1]) < 0.1:
                            temp_collect = 4000
                            print("mais de uma vítima")
                            break
                    else:
                        if victim_print:   
                            print("achei vítima:", v[1], v[2])
                        if abs(ang - total_gyro) < 0.1 or 2*math.pi-abs(ang - total_gyro) < 0.1:
                            action_list.append(('null', 30))
                        action_list.append(('collect', v[1], v[2], last_gps[0], last_gps[1]))
                        victim_done.append(v)
                        victim_remove = cont
                        q = [a, b]
            
            elif v[0] and q[0] == 0:
                if on_area_4 and map4.mark[wallb4, walla4] and map4.mark[wallb4_2, walla4_2] and map4.mark[wallb4_3, walla4_3]:
                    victim_list[cont][0] = 0
                    actualX, actualY = actualX4, actualY4

                    if emergencial_print:   
                        print("emergenciaaa4")
                        print(actualX, actualY, walla4, wallb4)

                    if bfs_4(actualX, actualY, walla4, wallb4):
                        backtracking = True
                        if emergencial_print:   
                            print("bfs para vítima emergencial")
                        [x, y] = [actualX, actualY]
                        invert_parent(walla4, wallb4, x, y)
                        # print("back")
                        cont = 0
                        while([x, y] != [walla4, wallb4]):
                            #print("action", parent2[x, y])
                            action_list.append(("walk_to", map4.tile_to_coords(parent2[x, y][0], parent2[x, y][1])[0], map4.tile_to_coords(parent2[x, y][0], parent2[x, y][1])[1], map4.tile_to_coords(x, y)[0], map4.tile_to_coords(x, y)[1]))
                            [x, y] = parent2[x, y]
                            if emergencial_print and bfs_print:   
                                print(x, y)
                        backwardX_4.put(actualX)
                        backwardY_4.put(actualY)
                        break
                    elif bfs_4(actualX, actualY, walla4_2, wallb4_2):
                        backtracking = True
                        if emergencial_print:   
                            print("bfs para vítima emergencial")
                        [x, y] = [actualX, actualY]
                        invert_parent(walla4_2, wallb4_2, x, y)
                        # print("back")
                        cont = 0
                        while([x, y] != [walla4_2, wallb4_2]):
                            #print("action", parent2[x, y])
                            action_list.append(("walk_to", map4.tile_to_coords(parent2[x, y][0], parent2[x, y][1])[0], map4.tile_to_coords(parent2[x, y][0], parent2[x, y][1])[1], map4.tile_to_coords(x, y)[0], map4.tile_to_coords(x, y)[1]))
                            [x, y] = parent2[x, y]
                            if emergencial_print and bfs_print:   
                                print(x, y)
                        backwardX_4.put(actualX)
                        backwardY_4.put(actualY)
                        break
                    elif bfs_4(actualX, actualY, walla4_3, wallb4_3):
                        backtracking = True
                        if emergencial_print:   
                            print("bfs para vítima emergencial")
                        [x, y] = [actualX, actualY]
                        invert_parent(walla4_3, wallb4_3, x, y)
                        # print("back")
                        cont = 0
                        while([x, y] != [walla4_3, wallb4_3]):
                            #print("action", parent2[x, y])
                            action_list.append(("walk_to", map4.tile_to_coords(parent2[x, y][0], parent2[x, y][1])[0], map4.tile_to_coords(parent2[x, y][0], parent2[x, y][1])[1], map4.tile_to_coords(x, y)[0], map4.tile_to_coords(x, y)[1]))
                            [x, y] = parent2[x, y]
                            if emergencial_print and bfs_print:   
                                print(x, y)
                        backwardX_4.put(actualX)
                        backwardY_4.put(actualY)
                        break
                    else:
                        victim_remove = cont
                elif not on_area_4 and map.mark[wallb, walla] and map.mark[wallb_2, walla_2] and map.mark[wallb_3, walla_3] and not (map4.mark[wallb4, walla4] or map4.mark[wallb4_2, walla4_2] or map4.mark[wallb4_3, walla4_3]):
                    victim_list[cont][0] = 0
                    actualX, actualY = map.coords_to_even_tile(last_gps[0], last_gps[1])
                    if emergencial_print:   
                        print("emergenciaaa")
                        print(actualX, actualY, walla, wallb)
                    if bfs(actualX, actualY, walla, wallb):
                        backtracking = True
                        if emergencial_print:   
                            print("bfs para vítima emergencial")
                        [x, y] = [actualX, actualY]
                        invert_parent(walla, wallb, x, y)
                        # print("back")
                        cont = 0
                        while([x, y] != [walla, wallb]):
                            #print("action", parent2[x, y])
                            action_list.append(("walk_to", map.tile_to_coords(parent2[x, y][0], parent2[x, y][1])[0], map.tile_to_coords(parent2[x, y][0], parent2[x, y][1])[1], map.tile_to_coords(x, y)[0], map.tile_to_coords(x, y)[1]))
                            [x, y] = parent2[x, y]
                            if emergencial_print and bfs_print:   
                                print(x, y)
                        backwardX.put(actualX)
                        backwardY.put(actualY)
                        break
                    if bfs(actualX, actualY, walla_2, wallb_2):
                        backtracking = True
                        if emergencial_print:   
                            print("bfs para vítima emergencial")
                        [x, y] = [actualX, actualY]
                        invert_parent(walla_2, wallb_2, x, y)
                        # print("back")
                        cont = 0
                        while([x, y] != [walla_2, wallb_2]):
                            #print("action", parent2[x, y])
                            action_list.append(("walk_to", map.tile_to_coords(parent2[x, y][0], parent2[x, y][1])[0], map.tile_to_coords(parent2[x, y][0], parent2[x, y][1])[1], map.tile_to_coords(x, y)[0], map.tile_to_coords(x, y)[1]))
                            [x, y] = parent2[x, y]
                            if emergencial_print and bfs_print:   
                                print(x, y)
                        backwardX.put(actualX)
                        backwardY.put(actualY)
                        break
                    elif bfs(actualX, actualY, walla_3, wallb_3):
                        backtracking = True
                        if emergencial_print:   
                            print("bfs para vítima emergencial")
                        [x, y] = [actualX, actualY]
                        invert_parent(walla_3, wallb_3, x, y)
                        # print("back")
                        cont = 0
                        while([x, y] != [walla_3, wallb_3]):
                            #print("action", parent2[x, y])
                            action_list.append(("walk_to", map.tile_to_coords(parent2[x, y][0], parent2[x, y][1])[0], map.tile_to_coords(parent2[x, y][0], parent2[x, y][1])[1], map.tile_to_coords(x, y)[0], map.tile_to_coords(x, y)[1]))
                            [x, y] = parent2[x, y]
                            if emergencial_print and bfs_print:   
                                print(x, y)
                        backwardX.put(actualX)
                        backwardY.put(actualY)
                        break
                    else:
                        victim_remove = cont
            

            cont = cont + 1
        if victim_remove != -1:
            victim_list.pop(victim_remove)

    print(backwardX.qsize() + backwardX_4.qsize()/3)
    if remaining_time < (backwardX.qsize() + backwardX_4.qsize()/3) + 5:
        backspawn()

    if len(action_list) == 0:
        if(on_area_4):
            x, y = map4.coords_to_tile(last_gps[0], last_gps[1])
            solve_4(x, y)
        else:
            solve(x, y)


# -------------------------------------------------------------------------------


def cleanup():
    global tick_count
    global current_tick
    global last_gps
    global acabou
    global dir_tick
    global qtd_prob

    tick_count += 1
    dir_tick += 1
    current_tick += 1

    if tick_count < 2:
        print(last_gps[0], last_gps[1])

    wheel_left.setVelocity(speeds[0])
    wheel_right.setVelocity(speeds[1])

    x, y = map.coords_to_tile(last_gps[0], last_gps[1])
    if(not on_area_4):
        if map.mark[y, x] <= 0 and y % 4 == 0 and x % 4 == 0:
            #print("mark", x, y)
            if not backtracking:
                for aux in range(0, 16):
                    X = x+h[aux]
                    Y = y+v[aux]

                    if map.mark[Y][X] != 0 and verify_wall(x, y, X, Y) and map.mark[int((Y+y)/2), int((X+x)/2)] == 0:
                        #print("MARK ARREDORES", int((X+x)/2), int((Y+y)/2))
                        map.mark[int((Y+y)/2), int((X+x)/2)] = -1
    else:
        x4, y4 = map4.coords_to_tile(last_gps[0], last_gps[1])

        if map4.mark[y4, x4] <= 0:
            #print("mark", x, y)
            for i in range(0, 5):
                for j in range(0, 5):
                    if not map4.mark[y4+2-i, x4+2-j]: 
                        map4.mark[y4+2-i, x4+2-j] = -1
            map4.mark[y4, x4] = 1
            if not backtracking:
                for aux in range(0, 24):
                    X = x4+h_4[aux]
                    Y = y4+v_4[aux]

                    if ((map4.mark[Y][X] != 0 and verify_wall_4(x4, y4, X, Y)) or (aux % 3 != 0 and map4.matrix[Y][X] == 1)):
                        if map4.mark[int((Y+y4)/2), int((X+x4)/2)] == 0:    
                            map4.mark[int((Y+y4)/2), int((X+x4)/2)] = -1
                        if aux % 3 == 0 and map4.mark[int((3*Y+y4)/4), int((3*X+x4)/4)] == 0:
                            map4.mark[int((3*Y+y4)/4), int((3*X+x4)/4)] = -1
    map.mark[y, x] = 1

    if qtd_prob >= 6:
        print("muito prob")
        qtd_prob = 0
        call_lop()

    if map.coords_to_tile(0, 0) == map.coords_to_tile(last_gps[0], last_gps[1]) and (semtempo or (tick_count > 500 and len(action_list) == 0)):
        print("Opa, to no começo")
        acabou = True
        map.end_map()

    if remaining_time == 3: 
        print("paro paro")
        map.end_map()


# Loop principal do código

while robot.step(time_step) != -1:
    if call_tick:
        call_lop()
        tick_count = tick_count+1
    elif not ended:
        update_sensors()

        if len(action_list) > 0:
            if not isinstance(action_list[next_action], Acao):
                # print(action_list[next_action])
                action_list[next_action] = Acao(action_list[next_action])

            if action_list[next_action].is_done():
                next_action += 1
                current_tick = 0
                if next_action >= len(action_list):
                    action_list = []
                    next_action = 0
            else:
                action_list[next_action].execute()
        if len(action_list) == 0:
            find_action()

        cleanup()