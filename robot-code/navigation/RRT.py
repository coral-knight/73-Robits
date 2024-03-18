import math
import random
import numpy as np

class RRT:

    def __init__(self, map, pos):
        self.map = map 
        #self.graph = [[pos, 0]]

        self.size = [2, 2]
        self.resolution = 0.06
        self.range_x = [-self.resolution, self.resolution]
        self.range_y = [-self.resolution, self.resolution]

        self.graph = np.empty(self.size, dtype=object)
        for x in range(self.size[0]):
            for y in range(self.size[1]):
                self.graph[x, y] = [0]


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
                self.graph = np.insert(self.graph, np.size(self.magraphp, 0), None, axis=0)
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


    def random_point(self):
        # random point between map boundaries
        random.seed()
        x_percent = random.random()
        y_percent = random.random()
        x = self.map.range_x[0]*x_percent + self.map.range_x[1]*(1-x_percent)
        y = self.map.range_y[0]*y_percent + self.map.range_y[1]*(1-y_percent)
        self.graph_expand([x, y])
        return [x, y]


    def closest_point(self, point):
        closest = [1000,1000]
        pos = 0
        cont = 0

        mapx, mapy = self.real_to_map(point)
        depth = 0
        # achar closest e pensar no q fazer com o pos dele
        #closest = 
        #while

        for v in self.graph:
            if self.dist_coords(closest, point) > self.dist_coords(v[0], point):
                closest = v[0]
                pos = cont
            cont += 1

        return [closest, pos]


    def project_point(self, point, closest):
        if self.dist_coords(point, closest) > 0.08:
            ang = math.atan2(point[1]-closest[1], point[0]-closest[0])
            point = [closest[0]+0.08*math.cos(ang), closest[1]+0.08*math.sin(ang)]

        return point


    def add_graph(self, point, closest, pos):
        # adiciona 'point' para o 'closest': [ponto (coordenada), pai (posição)]

        if not self.wall_between(closest, point):
            self.graph.append([point, pos])
            return True

        return False


    def explore(self, ticks):
        unexplored = []

        while ticks > 0:
            ticks -= 1

            point = self.random_point()
            [closest, pos] = self.closest_point(point)
            point = self.project_point(point, closest)
            added = self.add_graph(point, closest, pos)

            if added:
                map_p = self.map.real_to_map(point)
                if self.map.seen_map[map_p[0], map_p[1]] & 2 == 0:
                    unexplored.append([point, len(self.graph)-1]) # [ponto (coordenada), posição]

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


