from controller import Robot as Hardware
from robot.sensors import Sensors
from mapping.map import Map
from navigation.RRT import RRT
from navigation.RRTStar import RRTStar
from navigation.navigation import Navigate
import numpy as np
import math
import struct

class Robot:
    '''
    General robot controller
    '''
    
    def __init__(self, time_step):
        self.time_step = time_step
        
        self.calibration_timer = 50
        self.current_tick = 0

        self.hardware = Hardware()

        self.map = Map()
        
        self.sensors = Sensors(self.hardware, self.time_step, self.map)

        self.navigate = Navigate(self.hardware, self.sensors, self.map)
        
        #Receiver
        self.receiver = self.hardware.getDevice("receiver")
        self.receiver.enable(self.time_step)

        #Emmiter
        self.emitter = self.hardware.getDevice("emitter")

        self.exploring = False


    def send_victim(self, victim_position, victim_type):
        '''
        Sends victim information to server
        '''
        #message = struct.pack("i i c", int(victim_position[0], victim_position[1]), victim_type)
        #self.emitter.send(message)
        return


    def send_endgame(self, map):
        '''
        Sends endgame message and map to server
        '''
        #message = struct.pack("i i c", int((self.final_coords[0]+initial_gps[0])*100), int((self.final_coords[1]+initial_gps[1])*100), victim_type)
        #self.emitter.send(message)        
        return 


    def run_calibration(self):
        '''
        Starts the simulation by calibrating the robot 
        '''

        if self.current_tick == 1:
            self.sensors.gps.calibrate()

        self.sensors.gps.update()

        if self.current_tick < 5:
            self.navigate.speed(2, 2)
        elif self.current_tick == 5:
            self.sensors.gyro.calibrate(self.sensors.gps.last)
        elif self.current_tick < 10:
            self.navigate.speed(-2, -2)
        else:
            self.navigate.speed(0, 0)
            self.sensors.update(self.current_tick)
        return


    def run_simulation(self):
        '''
        Runs a tick of the robot simulation
        '''

        self.sensors.update(self.current_tick)

        if self.current_tick == self.calibration_timer+1:
            self.navigate.speed(0,0)
            self.global_rrt = RRTStar(self.map, self.sensors.gps.last)
            return
        

        # TEST
        if self.current_tick == 300:
            print("adicionei a vítima")
            self.sensors.camera.sign_list.append((10, [0.24, 0.06], [0.22, 0.06], [0.26, 0.06]))


        # Check if there's signs that the robot can safely go walking a straight path
        if not self.navigate.exploring:
            for sign in self.sensors.camera.sign_list:
                # sign = [[x_right-x_left], [pos], [left point pos], [right point pos]]

                # Create [x, y] slightly away from the sign's wall (vectors)
                [a, b] = sign[1]
                [ae, be] = sign[2]
                [ad, bd] = sign[3]

                vi = [ad-ae, bd-be]
                u = [-1*vi[1], -1*vi[0]]

                x = u[0] * (0.06 / math.sqrt(vi[0]**2 + vi[1]**2)) + a
                y = u[1] * (0.06 / math.sqrt(vi[0]**2 + vi[1]**2)) + b 

                # Robot's angle to the sign and wall angulation
                ang = math.atan2(b - self.sensors.gps.last[1], a - self.sensors.gps.last[0])
                ang_min = math.atan2(bd-be, ad-ae)
                ang_max = math.atan2(be-bd, ae-ad)  

                print("sign", sign[1], [x, y])
                print("angles", ang, ang_min, ang_max)

                # Check **if it's on the right side of the wall** and if there's no wall between
                if ((ang_min >= 0 and (ang > ang_min or ang < ang_max)) or (ang_min < 0 and ang > ang_min and ang < ang_max)) and (abs(ang-ang_min) > 0.47 and 2*math.pi-abs(ang-ang_min) > 0.47) and (abs(ang-ang_max) > 0.47 and 2*math.pi-abs(ang-ang_max) > 0.47):
                    print("right angle")
                    if not self.wall_between(self.sensors.gps.last, [x, y]):
                        print("sem parede")
                        self.navigate.exploring = True
                        self.navigate.append_list([x, y], 2)
                        break


        # Find a new point on the RRTs to go 
        if not self.navigate.exploring:
            self.navigate.speed(0, 0)
            print("------------------------------------")

            #self.global_rrt.update(self.sensors.gps.last, 1)

            print("new RRT")

            local_rrt = RRTStar(self.map, self.sensors.gps.last)

            local_unexplored = []
            global_unexplored = []

            cont = 0
            while len(local_unexplored) == 0 and len(global_unexplored) == 0 and cont < 1000:
                cont += 1
                local_unexplored = local_rrt.explore(10)
                global_unexplored = self.global_rrt.explore(1)

            if cont == 1000: 
                print("n achou nada")
                print("volta spawn")
                self.navigate.solve([[0, 0], self.global_rrt.real_to_pos([0, 0])], self.global_rrt.graph, [self.sensors.gps.last, self.global_rrt.cur_tile])

            elif len(local_unexplored) > 0:
                print("found LOCAL unexplored")
                print(local_unexplored[0])
                self.navigate.solve([local_unexplored[0], local_rrt.real_to_pos(local_unexplored[0])], local_rrt.graph, [self.sensors.gps.last, local_rrt.real_to_pos(self.sensors.gps.last)])

            elif len(global_unexplored) > 0:
                print("found GLOBAL unexplored")
                print(global_unexplored[0])
                self.navigate.solve([global_unexplored[0], self.global_rrt.real_to_pos(global_unexplored[0])], self.global_rrt.graph, [self.sensors.gps.last, self.global_rrt.real_to_pos(self.sensors.gps.last)])


        # Navigate
        if self.navigate.exploring:
            #self.global_rrt.update(self.sensors.gps.last, 0)
            action_list = self.navigate.navigate()
            for action in action_list:
                if action[0] == "delete":
                    print("pedido de deleta para", action[1])
                    self.global_rrt.delete(action[1])
                if action[0] == "connect":
                    print("pedido de ligação", self.sensors.gps.last, action[1])
                    self.global_rrt.connect(self.sensors.gps.last, action[1])
                if action[0] == "collect":
                    print("pedido de coleta")
                    self.sensors.camera.collect(self.navigate)
                
                if action[0] == "exit":
                    definitive_map = np.array(self.map.print_tile_map())
                    print("definitive_map")
                    print(definitive_map)
                    d = definitive_map.shape
                    d_bytes = struct.pack('2i', *d)
                    flatMap = ','.join(definitive_map.flatten())
                    dub_bytes = flatMap.encode('utf-8')
                    final_bytes = d_bytes + dub_bytes
                    self.emitter.send(final_bytes)
                    map_evaluate_request = struct.pack('c', b'M')
                    self.emitter.send(map_evaluate_request)
                    exit_mes = struct.pack('c', b'E')
                    self.emitter.send(exit_mes)
        
        return
        

    def run(self):
        '''
        Starts the simulation by calibrating the robot and setting the current direction
        '''
        while self.hardware.step(self.time_step) != -1:
            self.current_tick += 1
            if self.current_tick <= self.calibration_timer:
                self.run_calibration()
            else:
                self.run_simulation()
        return
    

    '''========================================= AUXILIAR FUNCTIONS ==========================================='''


    def dist_coords(self, a, b):
        dist = ((a[0]-b[0])**2 + (a[1]-b[1])**2)**0.5
        return dist


    def wall_between(self, a, b):
        # Can the robot go safely from a to b?

        d = int(self.dist_coords(a, b)/0.06)*3
        if d == 0: d = 1

        for i in range(d+1): 
            p = [((d-i)*a[0]+i*b[0])/d, ((d-i)*a[1]+i*b[1])/d]
            map_p = self.map.real_to_map(p)

            for x in range(-1, 2):
                for y in range(-1, 2):
                    if map_p[0]+x >= 0 and map_p[1]+y >= 0 and map_p[0]+x < np.size(self.map.map, 0) and map_p[1]+y < np.size(self.map.map, 1):
                        for v in self.map.map[map_p[0]+x, map_p[1]+y]:
                            if v != 0 and self.dist_coords(p, v) < 0.035:
                                print("parede", v, self.dist_coords(p, v))
                                return True
                
        return False
                