from controller import Robot as Hardware
from robot.process_sensors import Sensors
from mapping.map import Map
from navigation.RRT import RRT
from navigation.RRTStar import RRTStar
from navigation.navigation import Navigate
import numpy as np

class Robot:
    '''
    General robot controller
    '''
    
    def __init__(self, time_step):
        self.time_step = time_step
        
        self.calibration_timer = 10
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
            self.sensors.initial_gps = self.sensors.gps.getValues()

        self.sensors.update_gps()

        if self.current_tick < 5:
            self.navigate.speed(2, 2)
        elif self.current_tick == 5:
            self.sensors.calibrate_gyro()
        else:
            self.navigate.speed(-2, -2)
        return


    def run_simulation(self):
        '''
        Runs a tick of the robot simulation
        '''
        #print("=========================")

        self.sensors.update(self.current_tick)
        if self.current_tick < 30:
            self.navigate.speed(0,0)
            return

        #print("exploring:", self.navigate.exploring)

        if not self.navigate.exploring:
            self.navigate.speed(0,0)
            self.rrt = RRTStar(self.map, self.sensors.last_gps)

            print("------------------------------------")
            print("new RRT")

            unexplored = []
            while len(unexplored) == 0:
                #print("while no unexplored")
                #print("========================================================================")
                [unexplored, graph] = self.rrt.explore(5)

            print("found unexplored")
            print(unexplored[0])

            '''
            [unexplored, graph] = self.rrt.explore(25)

            print("--------")
            for x in range(np.size(graph, 0)):
                print("[", end = " ")
                for y in range(np.size(graph, 1)):
                    print(graph[x][y], end = "")
                    if y != np.size(graph, 1)-1:
                        print(",", end = " ")
                if x != np.size(graph, 0)-1:
                    print("]", end = "")
                    print(",", end = " ")
                else:
                    print("]")
            print("--------")
            '''
            
            self.navigate.solve(unexplored[0], graph)

        if self.navigate.exploring:
            self.navigate.navigate()

        '''
        if len(self.rrt_local):
            # caminho
        if len(self.walk_action) == 0:
            while True:
                if len(self.rrt_local) or len(self.rrt_global):
                    #caminho
                self.rrt.explore(5)
        else:
            # anda e chega se chegou
        '''
        
        return
        

    def run_endgame(self):
        '''
        Does the map cleanup and ends the simulation
        '''
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
        self.run_endgame()
        return
                