import math
import numpy as np

class Navigate:

    def __init__(self, hardware, sensors, map):
        self.hardware = hardware
        self.sensors = sensors
        self.map = map

        self.velocity = 5
        self.turn_velocity = 3
        self.cur_velocity = [0,0]

        self.exploring = False
        self.action_list = []
        self.last_walk = [[0,0], self.sensors.gps.last]

        #Left wheel
        self.wheel_left = self.hardware.getDevice("wheel1 motor")
        self.wheel_left.setPosition(float("inf"))

        #Right wheel
        self.wheel_right = self.hardware.getDevice("wheel2 motor")
        self.wheel_right.setPosition(float("inf"))


    def speed(self, left_speed, right_speed):
        
        self.wheel_left.setVelocity(left_speed)
        self.wheel_right.setVelocity(right_speed)
        self.cur_velocity = [left_speed, right_speed]
        return
    

    def walk_to(self, point, arg):
        ang = math.atan2(point[1]-self.sensors.gps.last[1], point[0]-self.sensors.gps.last[0])
        delta_angle = ang-self.sensors.gyro.last

        if abs(delta_angle) >= 0.05:
            #print("rotating", delta_angle)
            #print(ang, self.sensors.last_gyro)

            while delta_angle < -math.pi:
                delta_angle = delta_angle + 2*math.pi
            while delta_angle > math.pi:
                delta_angle = delta_angle - 2*math.pi
            if delta_angle >= 0:
                self.speed(self.turn_velocity, -self.turn_velocity)
            else:
                self.speed(-self.turn_velocity, self.turn_velocity)
        
        else:
            #print("walking", point)
            #print(self.sensors.gps.last)
            #print(self.dist_coords(self.sensors.gps.last, point))

            if self.dist_coords(self.sensors.gps.last, point) > 0.005:
                self.speed(self.velocity, self.velocity)
            else:
                print("terminou andar, argument: ", arg)
                self.last_walk[0] = self.last_walk[1]
                self.last_walk[1] = point
                self.action_list.pop(0)
                if self.dist_coords(self.sensors.last_gps, [0, 0]) <= 0.005: return[["exit"]]
                if arg == 0: return [["connect", self.last_walk[0]]] # new area
                if arg == 1: return [["nothing"]] # backtracking
                if arg == 2: return [["collect"]] # collectable sign

        return [["nothing"]]


    def navigate(self):
        #print("navigating")

        if len(self.action_list) <= 0:
            self.exploring = False

        else:
            name = self.action_list[0][0]
            action = self.action_list[0][1]
            arg = self.action_list[0][2]

            if name == "Walk To":
                if arg == False and self.wall_between(self.sensors.gps.last, action):
                    print("tinha parede no caminho que eu n vi para", action)
                    self.exploring = False
                    self.action_list = []
                    self.last_walk[0] = self.last_walk[1]
                    self.last_walk[1] = self.sensors.gps.last
                    print("atual e last:", self.last_walk[1], self.last_walk[0])
                    return [["delete", action], ["connect", self.last_walk[0]]]
                
                return self.walk_to(action, arg)

        return [["nothing"]]


    def append_list(self, point, arg):
        # arg == 0: has not walked that path (new areas) | arg == 1: has walked that path (backtracking) | arg == 2: collectable sign
        print("append walk_to", point)
        self.action_list.append(["Walk To", point, arg])
        return
    

    def path_smoothing(self):
        p, v = 0, 0
        aux_list = [["Walk To", self.sensors.gps.last]]

        while v != len(self.action_list):
            print("smoothing", p, v)
            a, b = self.action_list[p][1], self.action_list[v][1]
            if not self.wall_between(a, b):
                v += 1
            else:
                if p == v-1: return
                aux_list.append(self.action_list[v-1])
                p = v-1

        aux_list.append(self.action_list[v-1])
        self.action_list = aux_list[1:]

        return
    
    
    def solve(self, unexplored, graph, last):
        print("solve para", unexplored)
        print("last", last)
        self.exploring = True

        point = last[0]
        pos = last[1]
        print("teste", pos, graph[pos[0][0]][pos[0][1]])
        level = self.total_level(graph, pos)
        walk_list = []

        unpoint = unexplored[0]
        unpos = unexplored[1]
        unlevel = self.total_level(graph, unpos)
        unwalk_list = []

        print(pos, unpos)
        print(level, unlevel)

        a = 0
        while level > unlevel and a < 1000:
            pos = self.graph_parent(graph, pos) # Graph position of the parent
            point = graph[pos[0][0]][pos[0][1]][pos[1]][0] # Coordinates of the parent

            print("add walk", point)
            walk_list.insert(0, point)

            level -= 1

            a += 1
        if a == 1000: print("WHILE WALK_LIST")

        b = 0
        while unlevel > level and b < 1000:
            print("add unwalk", unpoint)
            unwalk_list.append(unpoint)

            unpos = self.graph_parent(graph, unpos) # Graph position of the parent
            unpoint = graph[unpos[0][0]][unpos[0][1]][unpos[1]][0] # Coordinates of the parent

            unlevel -= 1

            b += 1
        if b == 1000: print("WHILE UNWALK_LIST")

        print("level igual")

        cont = 0
        while point != unpoint and cont < 1000:
            print("add unwalk", unpoint)
            unwalk_list.append(unpoint)

            unpos = self.graph_parent(graph, unpos)
            unpoint = graph[unpos[0][0]][unpos[0][1]][unpos[1]][0] 

            pos = self.graph_parent(graph, pos)
            point = graph[pos[0][0]][pos[0][1]][pos[1]][0] 

            print("add walk", point)
            walk_list.insert(0, point)

            cont += 1
        if cont == 1000: print("WHILE TOGETHER SOLVE")

        print("walk do unexplored")
        for w in unwalk_list:
            print(w)

        print("walk do atual")
        for w in walk_list:
            print(w)

        #walk_list = unwalk_list + walk_list

        self.append_list(self.sensors.gps.last, 1)
        for i in range(len(walk_list)-1, -1, -1):
            self.append_list(walk_list[i], 1)

        for i in range(len(unwalk_list)-1, -1, -1):
            self.append_list(unwalk_list[i], 0)

        self.path_smoothing()

        return
    

    '''========================================= AUXILIAR FUNCTIONS ==========================================='''
    

    def dist_coords(self, a, b):
        dist = ((a[0]-b[0])**2 + (a[1]-b[1])**2)**0.5
        return dist
    

    def graph_parent(self, graph, pos):
        # Get the position of the parent of a graph point on the graph

        parent_pos = [[0, 0], 0]
        parent_pos[0][0] = pos[0][0] + graph[pos[0][0]][pos[0][1]][pos[1]][1][0][0]
        parent_pos[0][1] = pos[0][1] + graph[pos[0][0]][pos[0][1]][pos[1]][1][0][1]
        parent_pos[1] = graph[pos[0][0]][pos[0][1]][pos[1]][1][1]

        return parent_pos

    
    def total_level(self, graph, pos):
        # Get the level of a graph point

        s = 0
        parent_pos = self.graph_parent(graph, pos)

        while pos != parent_pos and s < 500:
            s += 1
            pos = parent_pos
            parent_pos = self.graph_parent(graph, pos) # Graph position of point parent

        if s == 500: print("WHILE TOTAL_LEVEL")

        return s
    

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