import math
import random
import numpy as np

class RRT:

    def __init__(self, map, pos):
        self.map = map 
        self.graph = [[pos, 0]]


    def random_point(self):
        # random point between map boundaries
        random.seed()
        x_percent = random.random()
        y_percent = random.random()
        x = self.map.range_x[0]*x_percent + self.map.range_x[1]*(1-x_percent)
        y = self.map.range_y[0]*y_percent + self.map.range_y[1]*(1-y_percent)
        return [x,y]


    def closest_point(self, point):
        closest = [1000,1000]
        pos = 0
        cont = 0
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


    def add_graph(self, point):
        # adiciona 'point' para o 'closest': [ponto (coordenada), pai (posição)]
        [closest, pos] = self.closest_point(point)
        point = self.project_point(point, closest)

        if not self.wall_between(point, closest):
            self.graph.append([point, pos])

        return 


    def explore(self, ticks):
        unexplored = []

        while ticks > 0:
            ticks -= 1

            point = self.random_point()
            added = self.add_graph(point)

            if added:
                map_p = self.map.real_to_map(point)
                if map.seen_map[map_p[0], map_p[1]] == 0:
                    unexplored.append([point, len(self.graph)-1]) # [ponto (coordenada), posição]

        return [unexplored, self.graph]


    def dist_coords(self, a, b):
        dist = ((a[0]-b[0])**2 + (a[1]-b[1])**2)**0.5
        return dist
    

    def wall_between(self, a, b):
        for i in range(20): 
            p = [(i*a[0]+(19-i)*b[0])/19, (i*a[1]+(19-i)*b[1])/19]
            map_p = self.map.real_to_map(p)

            for x in range(-1, 2):
                for y in range(-1, 2):
                    if map_p[0]+x >= 0 and map_p[1]+y >= 0 and map_p[0]+x < np.size(self.map.map, 0) and map_p[1]+y < np.size(self.map.map, 1):
                        for v in self.map.map[map_p[0]+x, map_p[1]+y]:
                            if v != 0 and self.dist_coords(p, v) < 0.04:
                                return True
                
        return False


