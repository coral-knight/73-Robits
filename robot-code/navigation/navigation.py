import math
import numpy as np

class Navigate:

    def __init__(self, hardware, sensors, map):
        self.hardware = hardware
        self.sensors = sensors
        self.map = map

        self.velocity = 5
        self.turn_velocity = 3

        self.exploring = False
        self.action_list = []

        #Left wheel
        self.wheel_left = self.hardware.getDevice("wheel1 motor")
        self.wheel_left.setPosition(float("inf"))

        #Right wheel
        self.wheel_right = self.hardware.getDevice("wheel2 motor")
        self.wheel_right.setPosition(float("inf"))


    def speed(self, left_speed, right_speed):
        
        self.wheel_left.setVelocity(left_speed)
        self.wheel_right.setVelocity(right_speed)
        return
    

    def walk_to(self, point):
        ang = math.atan2(point[1]-self.sensors.last_gps[1], point[0]-self.sensors.last_gps[0])
        delta_angle = ang-self.sensors.last_gyro

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
            #print(self.sensors.last_gps)
            #print(self.dist_coords(self.sensors.last_gps, point))

            if self.dist_coords(self.sensors.last_gps, point) > 0.005:
                self.speed(self.velocity, self.velocity)
            else:
                print("terminou andar")
                self.action_list.pop(0)

        return


    def navigate(self):
        #print("navigating")

        if len(self.action_list) == 0:
            self.exploring = False

        else:
            name = self.action_list[0][0]
            action = self.action_list[0][1]

            if name == "Walk To":
                if self.wall_between(self.sensors.last_gps, action):
                    self.exploring = False
                    print("tinha parede no caminho que eu n vi", action)
                    self.action_list = []
                    return
                
                self.walk_to(action)

        return


    def make_list(self, point):
        print("append walk_to", point)
        self.action_list.append(["Walk To", point])
        return
    

    def path_smoothing(self):
        p, v = 0, 0
        aux_list = [["Walk To", self.sensors.last_gps]]

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
        print("solve para", unexplored[0])
        self.exploring = True

        point = last[0]
        pos = last[1]
        print(pos)
        print(graph[pos[0][0]][pos[0][1]])
        #print(graph[pos[0][0]-1][pos[0][1]-1])
        level = self.total_level(graph, pos)
        walk_list = []

        unpoint = unexplored[0]
        unpos = unexplored[1]
        print(unpos)
        print(graph[unpos[0][0]][unpos[0][1]])
        unlevel = self.total_level(graph, unpos)
        unwalk_list = []

        a = 0
        while level > unlevel and a < 1000:
            pos = self.graph_parent(graph, pos) # Graph position of the parent
            point = graph[pos[0][0]][pos[0][1]][pos[1]][0] # Coordinates of the parent

            walk_list.insert(0, point)

            level -= 1

            a += 1
        print("passou", a)

        b = 0
        while unlevel > level and b < 1000:
            unwalk_list.append(unpoint)

            unpos = self.graph_parent(graph, unpos) # Graph position of the parent
            unpoint = graph[unpos[0][0]][unpos[0][1]][unpos[1]][0] # Coordinates of the parent

            unlevel -= 1

            b += 1
        print("passou dois", b)

        cont = 0
        while point != unpoint and cont < 1000:
            print(unpoint)
            unwalk_list.append(unpoint)

            unpos = self.graph_parent(graph, unpos)
            unpoint = graph[unpos[0][0]][unpos[0][1]][unpos[1]][0] 

            pos = self.graph_parent(graph, pos)
            point = graph[pos[0][0]][pos[0][1]][pos[1]][0] 

            print(point)
            walk_list.insert(0, point)

            cont += 1
        print("cont", cont)

        walk_list = unwalk_list + walk_list

        self.make_list(self.sensors.last_gps)
        for i in range(len(walk_list)-1, -1, -1):
            self.make_list(walk_list[i])

        self.path_smoothing()

        return
    

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

        while pos != parent_pos:
            s += 1
            pos = parent_pos
            parent_pos = self.graph_parent(graph, pos) # Graph position of point parent

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
                            if v != 0 and self.dist_coords(p, v) < 0.0378:
                                print("parede", v, self.dist_coords(p, v))
                                return True
                
        return False