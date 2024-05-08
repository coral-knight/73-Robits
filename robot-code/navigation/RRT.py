import math
import random
import numpy as np

class RRT:

    def __init__(self, map, sensors):
        self.map = map 
        self.sensors = sensors
        pos = self.sensors.gps.last

        self.size = [2, 2]
        self.resolution = 0.06
        self.range_x = [pos[0]-self.resolution, pos[0]+self.resolution]
        self.range_y = [pos[1]-self.resolution, pos[1]+self.resolution]

        # graph[x,y] = [ [ [i,j] , [ [x,y], 0] ],...] -> [[coordenada], [[tile do pai], posição dentro do tile do pai]]
        self.graph = np.empty(self.size, dtype=object)
        for x in range(self.size[0]):
            for y in range(self.size[1]):
                self.graph[x, y] = [0]
        self.graph[self.real_to_map(pos)[0], self.real_to_map(pos)[1]].append([pos, [[0,0], 0]])


    def graph_expand(self, point):
        if point[0] < self.range_x[0]:
            dif_map_x = math.ceil((self.range_x[0] - point[0])/self.resolution)
            while dif_map_x:
                self.graph = np.insert(self.graph, 0, None, axis=0)
                for y in range(np.size(self.graph, 1)): self.graph[0, y] = [0]
                self.range_x[0] = self.range_x[0] - self.resolution
                dif_map_x = dif_map_x - 1

        if point[0] > self.range_x[1]: 
            dif_map_x = math.ceil((point[0] - self.range_x[1])/self.resolution)
            while dif_map_x:
                self.graph = np.insert(self.graph, np.size(self.graph, 0), None, axis=0)
                for y in range(np.size(self.graph, 1)): self.graph[np.size(self.graph, 0)-1, y] = [0]
                self.range_x[1] = self.range_x[1] + self.resolution
                dif_map_x = dif_map_x - 1

        if point[1] < self.range_y[0]: 
            dif_map_y = math.ceil((self.range_y[0] - point[1])/self.resolution)
            while dif_map_y:
                self.graph = np.insert(self.graph, 0, None, axis=1)
                for x in range(np.size(self.graph, 0)): self.graph[x, 0] = [0]
                self.range_y[0] = self.range_y[0] - self.resolution
                dif_map_y = dif_map_y - 1

        if point[1] > self.range_y[1]: 
            dif_map_y = math.ceil((point[1] - self.range_y[1])/self.resolution)
            while dif_map_y:
                self.graph = np.insert(self.graph, np.size(self.graph, 1), None, axis=1)
                for x in range(np.size(self.graph, 0)): self.graph[x, np.size(self.graph, 1)-1] = [0]
                self.range_y[1] = self.range_y[1] + self.resolution
                dif_map_y = dif_map_y - 1

        return 
    

    def real_to_map(self, real_point):

        return [math.floor((real_point[0]-self.range_x[0])/self.resolution),
                math.floor((real_point[1]-self.range_y[0])/self.resolution)]
        

    def map_to_real(self, map_point):
        
        return [self.range_x[0]+map_point[0]*self.resolution+self.resolution/2,
                self.range_y[0]+map_point[1]*self.resolution+self.resolution/2]


    def random_point(self):
        # random point between map boundaries
        random.seed()
        x_percent = random.random()
        y_percent = random.random()
        x = self.range_x[0]*x_percent + self.range_x[1]*(1-x_percent)
        y = self.range_y[0]*y_percent + self.range_y[1]*(1-y_percent)
        return [x, y]


    def closest_point(self, point): # ===============
        mapx, mapy = self.real_to_map(point)
        #print("ponto no", mapx, mapy)

        closest = [1000,1000]
        pos = [[0,0], 0]
        depth = 0

        c = 0
        while closest == [1000,1000] and c < 50:
            c+=1
            #print("closest while", depth)
            for y in range(mapy-depth, mapy+depth+1):
                for x in range(mapx-depth, mapx+depth+1, (1 if y == mapy-depth or y == mapy+depth else 2*depth)):
                    #print("tiles", x, y)
                    if x >= 0 and x < np.size(self.graph, 0) and y >= 0 and y < np.size(self.graph, 1):
                        cont = 0
                        for v in self.graph[x, y]:
                            #print(x, y, ":", v)
                            if v != 0 and self.dist_coords(closest, point) > self.dist_coords(v[0], point):
                                closest = v[0]
                                pos = [[x, y], cont]
                            cont += 1
            depth += 1

        #print("closest", closest, pos)
        return [closest, pos]


    def project_point(self, point, closest):
        if self.dist_coords(point, closest) > 0.08:
            ang = math.atan2(point[1]-closest[1], point[0]-closest[0])
            point = [closest[0]+0.08*math.cos(ang), closest[1]+0.08*math.sin(ang)]

        return point


    def add_graph(self, point, closest, pos):
        # adiciona 'point' para o 'closest': [ponto (coordenada), pai (posição)]
        mapx, mapy = self.real_to_map(point)

        if not self.wall_between(closest, point):
            self.graph[mapx, mapy].append([point, pos])
            return True

        return False


    def explore(self, ticks):
        unexplored = []
        self.graph_expand([self.map.range_x[0]+0.03, self.map.range_y[0]-0.03])
        self.graph_expand([self.map.range_x[1]+0.03, self.map.range_y[1]-0.03])

        while ticks > 0:
            ticks -= 1

            point = self.random_point()
            [closest, pos] = self.closest_point(point)
            if closest == [1000,1000]: return [[], []]
            point = self.project_point(point, closest)
            added = self.add_graph(point, closest, pos)

            if added:
                map_p = self.map.real_to_map(point)
                # Sujeito a mudanças no '& 2 == 0' (por enquanto só verifica se não está marcado por 3)
                if self.map.seen_map[map_p[0], map_p[1]] & 2 == 0:
                    mapx, mapy = self.real_to_map(point)
                    unexplored.append([point, [[mapx, mapy], len(self.graph[mapx, mapy])-1]]) # [ponto (coordenada), posição]

        return [unexplored, self.graph]


    def dist_coords(self, a, b):
        dist = ((a[0]-b[0])**2 + (a[1]-b[1])**2)**0.5
        return dist
    

    def wall_between(self, a, b):
        # Can the robot go safely from a to b?

        for i in range(20): 
            p = [((19-i)*a[0]+i*b[0])/19, ((19-i)*a[1]+i*b[1])/19]
            map_p = self.map.real_to_map(p)

            for x in range(-1, 2):
                for y in range(-1, 2):
                    if map_p[0]+x >= 0 and map_p[1]+y >= 0 and map_p[0]+x < np.size(self.map.map, 0) and map_p[1]+y < np.size(self.map.map, 1):
                        for v in self.map.map[map_p[0]+x, map_p[1]+y]:
                            if v != 0 and self.dist_coords(p, v) < 0.0387:
                                return True
                
        return False


