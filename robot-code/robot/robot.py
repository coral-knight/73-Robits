from controller import Robot as Hardware
from robot.sensors import Sensors
from mapping.map import Map
from navigation.RRTStar import RRTStar
from navigation.navigation import Navigation
import numpy as np
import math
import struct

class Robot:
    '''
    General robot controller
    '''
    
    def __init__(self, time_step):
        self.time_step = time_step

        self.hardware = Hardware()
        self.map = Map()
        self.sensors = Sensors(self.hardware, self.time_step, self.map)
        self.navigation = Navigation(self.hardware, self.sensors, self.map)

        # VARIABLES
        self.current_tick = 0
        self.calibration_timer = 0
        self.calibrated = False
        self.c_initial_pos = [0, 0]
        self.c_total_gyro = 0
        self.c_last_tick_gyro = 0


    def run_calibration(self):
        '''
        Starts the simulation by calibrating the robot 
        '''
        self.calibration_timer += 1

        if self.calibration_timer == 1: self.c_initial_pos = self.sensors.gps.last

        if self.current_tick == 1: self.sensors.gps.calibrate()
        self.sensors.gps.update()

        if self.calibration_timer < 5:
            self.navigation.speed(2, 2)
        elif self.calibration_timer == 5:
            self.sensors.gyro.calibrate([self.sensors.gps.last[0]-self.c_initial_pos[0], self.sensors.gps.last[1]-self.c_initial_pos[1]])
            self.c_last_tick_gyro = self.sensors.gyro.last
            print("CALIBRATED GYRO", self.sensors.gyro.last)
        elif self.calibration_timer < 10:
            self.navigation.speed(-2, -2)
        else:
            self.sensors.update(self.current_tick)

            self.navigation.speed(2, -2)
            self.c_total_gyro += min(abs(self.sensors.gyro.last-self.c_last_tick_gyro), 2*math.pi-abs(self.sensors.gyro.last-self.c_last_tick_gyro))
            self.c_last_tick_gyro = self.sensors.gyro.last

            if self.c_total_gyro > 2*math.pi:
                self.calibrated = True
        return


    def run_simulation(self):
        '''
        Runs a tick of the robot simulation
        '''

        if self.navigation.check_LOP():
            self.calibration_timer = 0
            self.calibrated = False
            return
        
        self.sensors.update(self.current_tick)

        if self.current_tick == self.calibration_timer+1:
            self.navigation.speed(0,0)
            self.global_rrt = RRTStar(self.map, self.sensors.gps.last)
            return


        # Collect
        if self.navigation.collecting: 
            self.navigation.collecting = self.sensors.camera.collect(self.navigation, self.current_tick)


        # Check if there's signs that the robot can safely go walking a straight path
        if self.current_tick % 20 == 0 and not self.navigation.collecting and not self.navigation.walk_collect:
            for sign in self.sensors.camera.sign_list:
                # sign = [[x_right-x_left], [pos], [left point pos], [right point pos], [ang_min, ang_max]]

                # Create [x, y] slightly away from the sign's wall (vectors)
                [a, b] = sign[1]
                [ae, be] = sign[2]
                [ad, bd] = sign[3]

                vi = [ad-ae, bd-be]
                u = [vi[1], -1*vi[0]]

                w = [vi[0] * (0.04 / math.sqrt(vi[0]**2 + vi[1]**2)), vi[1] * (0.04 / math.sqrt(vi[0]**2 + vi[1]**2))]
                uw = [w[1], -1*w[0]]

                w2 = [-1*w[0], -1*w[1]]
                uw2 = [-1*w2[1], w2[0]]

                x = u[0] * (0.06 / math.sqrt(vi[0]**2 + vi[1]**2)) + a
                y = u[1] * (0.06 / math.sqrt(vi[0]**2 + vi[1]**2)) + b 

                x_right = uw[0] * (0.06 / math.sqrt(w[0]**2 + w[1]**2)) + a + w[0]
                y_right = uw[1] * (0.06 / math.sqrt(w[0]**2 + w[1]**2)) + b + w[1]

                x_left = uw2[0] * (0.06 / math.sqrt(w2[0]**2 + w2[1]**2)) + a + w2[0]
                y_left = uw2[1] * (0.06 / math.sqrt(w2[0]**2 + w2[1]**2)) + b + w2[1]

                # Robot's angle to the sign and wall angulation
                ang = math.atan2(b - self.sensors.gps.last[1], a - self.sensors.gps.last[0])
                [ang_min, ang_max] = sign[4] 

                print("sign", sign[1], [x, y])
                print("left", [ae, be])
                print("right", [ad, bd])
                print("angles", ang, ang_min, ang_max)

                # Check **if it's on the right side of the wall** and if there's no wall between
                if ((ang_min >= 0 and (ang > ang_min or ang < ang_max)) or (ang_min < 0 and ang > ang_min and ang < ang_max)) and (abs(ang-ang_min) > 0.47 and 2*math.pi-abs(ang-ang_min) > 0.47) and (abs(ang-ang_max) > 0.47 and 2*math.pi-abs(ang-ang_max) > 0.47):
                    #print("right angle")
                    if not self.wall_between(self.sensors.gps.last, [x, y]):
                        print("sem parede")
                        self.navigation.exploring = True
                        self.navigation.append_list([x, y], 2)
                        break


        # Find a new point on the RRTs to go 
        if not self.navigation.collecting and not self.navigation.exploring:
            self.navigation.speed(0, 0)
            print("------------------------------------")

            #self.global_rrt.update(self.sensors.gps.last, 1)

            print("new RRT")

            local_rrt = RRTStar(self.map, self.sensors.gps.last)

            local_unexplored = []
            global_unexplored = []

            cont = 0
            while len(local_unexplored) == 0 and len(global_unexplored) == 0 and cont < 500:
                cont += 1
                local_unexplored = local_rrt.explore(10)
                global_unexplored = self.global_rrt.explore(1)
                

            if len(local_unexplored) == 0 and len(global_unexplored) == 0: 
                print("n achou nada \n volta spawn")
                self.navigation.solve([[0, 0], self.global_rrt.real_to_pos([0, 0])], self.global_rrt.graph, [self.sensors.gps.last, self.global_rrt.cur_tile])

            elif len(local_unexplored) > 0:
                print("found LOCAL unexplored")
                print("cont", cont)
                print(local_unexplored[0])
                self.navigation.solve([local_unexplored[0], local_rrt.real_to_pos(local_unexplored[0])], local_rrt.graph, [self.sensors.gps.last, local_rrt.real_to_pos(self.sensors.gps.last)])

            elif len(global_unexplored) > 0:
                print("found GLOBAL unexplored")
                print(global_unexplored[0])
                self.navigation.solve([global_unexplored[0], self.global_rrt.real_to_pos(global_unexplored[0])], self.global_rrt.graph, [self.sensors.gps.last, self.global_rrt.real_to_pos(self.sensors.gps.last)])

        # Navigate
        if self.navigation.exploring:
            #self.global_rrt.update(self.sensors.gps.last, 0)
            action_list = self.navigation.navigate()
            for action in action_list:

                if action[0] == "delete":
                    print("pedido de deleta para", action[1])
                    self.global_rrt.delete(action[1])

                if action[0] == "connect":
                    print("pedido de ligação", self.sensors.gps.last, action[1])
                    self.global_rrt.connect(self.sensors.gps.last, action[1])

                if action[0] == "collect":
                    print("pedido de coleta")
                    self.navigation.collecting = True
                
                if action[0] == "exit":
                    definitive_map = np.array(self.map.print_tile_map())
                    print("definitive_map")
                    print(definitive_map)
                    d = definitive_map.shape
                    d_bytes = struct.pack('2i', *d)
                    flatMap = ','.join(definitive_map.flatten())
                    dub_bytes = flatMap.encode('utf-8')
                    final_bytes = d_bytes + dub_bytes
                    self.sensors.emitter.send(final_bytes)
                    map_evaluate_request = struct.pack('c', b'M')
                    self.sensors.emitter.send(map_evaluate_request)
                    exit_mes = struct.pack('c', b'E')
                    self.sensors.emitter.send(exit_mes)
        
        return
        

    def run(self):
        '''
        Starts the simulation by calibrating the robot and setting the current direction
        '''
        while self.hardware.step(self.time_step) != -1:
            self.current_tick += 1
            if not self.calibrated:
                self.run_calibration()
            else:
                self.run_simulation()
                #self.navigation.speed(0, 0)
                #self.sensors.update(self.current_tick)
            #if self.current_tick % 200 and self.current_tick > 50:
                #print(self.sensors.camera.identify_token(self.sensors.camera.joint_image()))
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
                