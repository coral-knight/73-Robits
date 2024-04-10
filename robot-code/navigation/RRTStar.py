import math
import random
import numpy as np

class RRTStar:

    def __init__(self, map, pos):
        self.map = map 

        self.size = [2, 2]
        self.resolution = 0.06
        self.min_dist = 0.08
        self.radius = 0.12
        self.range_x = [pos[0]-self.resolution, pos[0]+self.resolution]
        self.range_y = [pos[1]-self.resolution, pos[1]+self.resolution]

        # graph[x,y] = [ [ [i,j] , [ [x,y], 0], True], ...] -> [[coordenada], [[dist tiles pro pai], posição dentro do tile do pai], útil]
        self.graph = np.empty(self.size, dtype=object)
        for x in range(self.size[0]):
            for y in range(self.size[1]):
                self.graph[x, y] = [0]

        print("creating RRT", self.real_to_map(pos))
        self.graph[self.real_to_map(pos)[0], self.real_to_map(pos)[1]].append([pos, [[0, 0], 1], True])

        self.initial_pos = self.map_to_real(self.real_to_map(pos))
        self.cur_tile = [self.real_to_map(pos), 1]


    def graph_expand(self, point):
        if point[0] < self.range_x[0]:
            dif_map_x = math.ceil((self.range_x[0] - point[0])/self.resolution) + 1
            while dif_map_x:
                self.graph = np.insert(self.graph, 0, None, axis=0)
                for y in range(np.size(self.graph, 1)): self.graph[0, y] = [0]
                self.range_x[0] = self.range_x[0] - self.resolution
                self.cur_tile[0][0] += 1
                dif_map_x = dif_map_x - 1

        if point[0] > self.range_x[1]: 
            dif_map_x = math.ceil((point[0] - self.range_x[1])/self.resolution) + 1
            while dif_map_x:
                self.graph = np.insert(self.graph, np.size(self.graph, 0), None, axis=0)
                for y in range(np.size(self.graph, 1)): self.graph[np.size(self.graph, 0)-1, y] = [0]
                self.range_x[1] = self.range_x[1] + self.resolution
                dif_map_x = dif_map_x - 1

        if point[1] < self.range_y[0]: 
            dif_map_y = math.ceil((self.range_y[0] - point[1])/self.resolution) + 1
            while dif_map_y:
                self.graph = np.insert(self.graph, 0, None, axis=1)
                for x in range(np.size(self.graph, 0)): self.graph[x, 0] = [0]
                self.range_y[0] = self.range_y[0] - self.resolution
                self.cur_tile[0][1] += 1
                dif_map_y = dif_map_y - 1

        if point[1] > self.range_y[1]: 
            dif_map_y = math.ceil((point[1] - self.range_y[1])/self.resolution) + 1
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
        # Random point between map boundaries
        random.seed()
        x_percent = random.random()
        y_percent = random.random()
        x = self.range_x[0]*x_percent + self.range_x[1]*(1-x_percent)
        y = self.range_y[0]*y_percent + self.range_y[1]*(1-y_percent)
        return [x, y]
    

    def closest_point(self, point): 
        # Find the closest node to the point

        mapx, mapy = self.real_to_map(point)

        closest = [1000,1000]
        depth = 0

        c = 0
        while closest == [1000,1000] and c < 50:
            c+=1
            for y in range(mapy-depth, mapy+depth+1):
                for x in range(mapx-depth, mapx+depth+1, (1 if y == mapy-depth or y == mapy+depth else 2*depth)):
                    if x >= 0 and x < np.size(self.graph, 0) and y >= 0 and y < np.size(self.graph, 1):
                        for v in self.graph[x, y]:
                            if v != 0 and self.dist_coords(closest, point) > self.dist_coords(v[0], point) and v[2] == True:
                                closest = v[0]
            depth += 1

        return closest


    def project_point(self, point, closest):
        if self.dist_coords(point, closest) > self.min_dist:
            ang = math.atan2(point[1]-closest[1], point[0]-closest[0])
            point = [closest[0]+self.min_dist*math.cos(ang), closest[1]+self.min_dist*math.sin(ang)]

        return point


    def add_graph(self, point): # ===============
        # Find points with a distance from 'point' less than 'radius', adding the new node and updating neighbours distances

        mapx, mapy = self.real_to_map(point)

        neighbours = []
        parent = [1000, 1000]
        pos = [[0,0], 0]
        dist = 1000
        
        # Find neighbours and the parent
        depth = 0
        while depth < int(self.radius/self.resolution)+1:
            for y in range(mapy-depth, mapy+depth+1):
                for x in range(mapx-depth, mapx+depth+1, (1 if y == mapy-depth or y == mapy+depth else 2*depth)):
                    if x >= 0 and x < np.size(self.graph, 0) and y >= 0 and y < np.size(self.graph, 1):
                        cont = 0
                        for v in self.graph[x, y]:
                            if v != 0 and v[0] == point: return [1000, 1000], [[0,0], 0] 
                            if v != 0 and self.dist_coords(v[0], point) <= self.radius and not self.wall_between(v[0], point) and v[2] == True:
                                neighbours.append([v, cont])
                                if self.total_dist([[x, y], cont]) + self.dist_coords(v[0], point) < dist:
                                    parent = v[0]
                                    pos = [[x-mapx, y-mapy], cont]
                                    dist = self.total_dist([[x, y], cont]) + self.dist_coords(v[0], point)
                            cont += 1
            depth += 1

        # Add 'point' to graph 
        if parent == [1000, 1000]: return [1000, 1000], [[0,0], 0]
        self.graph[mapx, mapy].append([point, pos, True])

        # Change parent for neighbours if passing through the new node results in a smaller path
        for v in neighbours:
            [p, c] = v
            if p[0] != parent and dist + self.dist_coords(point, p[0]) < self.total_dist([[mapx, mapy], len(self.graph[mapx, mapy])-1]):
                x, y = self.real_to_map(p[0])
                self.graph[x][y][c][1] = [[mapx-x, mapy-y], len(self.graph[mapx, mapy])-1]

        return parent, [[mapx, mapy], len(self.graph[mapx, mapy])-1]
    

    def update(self, pos):
        # Add current position to the Global RRT Graph 

        closest = self.closest_point(pos)
        if self.dist_coords(pos, closest) > self.min_dist:
            self.add_graph(pos)

        return
    

    def delete(self, pos):
        x, y = self.real_to_map(pos)

        print("aqui", x, y)
        for v in self.graph[x, y]:
            print(v)
            if v != 0 and v[0] == pos:
                print("removendo", v)
                v[2] = False

        return 


    def explore(self, ticks):
        # Generate and find unexplored nodes to the navigation

        unexplored = []
        self.graph_expand([self.map.range_x[0], self.map.range_y[0]])
        self.graph_expand([self.map.range_x[1], self.map.range_y[1]])

        while ticks > 0:
            ticks -= 1

            point = self.random_point()
            closest = self.closest_point(point)
            point = self.project_point(point, closest)
            parent, pos = self.add_graph(point)
            if parent == [1000,1000]: continue

            map_p = self.map.real_to_map(point)
            if map_p[0] >= 0 and map_p[0] < np.size(self.map.map, 0) and map_p[1] >= 0 and map_p[1] < np.size(self.map.map, 1):
                if self.map.seen_map[map_p[0], map_p[1]] == 0:
                    print("inexplorado", point, pos, parent)
                    unexplored.append([point, pos]) # [ponto (coordenada), posição]

        return [unexplored, self.graph]


    def dist_coords(self, a, b):
        dist = ((a[0]-b[0])**2 + (a[1]-b[1])**2)**0.5
        return dist
    

    def graph_parent(self, pos):
        # Get the position of the parent of a graph point on the graph

        parent_pos = [[0, 0], 0]
        parent_pos[0][0] = pos[0][0] + self.graph[pos[0][0]][pos[0][1]][pos[1]][1][0][0]
        parent_pos[0][1] = pos[0][1] + self.graph[pos[0][0]][pos[0][1]][pos[1]][1][0][1]
        parent_pos[1] = self.graph[pos[0][0]][pos[0][1]][pos[1]][1][1]

        return parent_pos
    

    def total_dist(self, pos):
        # Get the distance from the current node to the initial node

        s = 0

        point = self.graph[pos[0][0]][pos[0][1]][pos[1]][0]

        parent_pos = self.graph_parent(pos)
        parent_point = self.graph[parent_pos[0][0]][parent_pos[0][1]][parent_pos[1]][0]

        while pos != parent_pos:

            s += self.dist_coords(point, parent_point)

            pos = parent_pos
            point = parent_point

            parent_pos = self.graph_parent(pos)
            parent_point = self.graph[parent_pos[0][0]][parent_pos[0][1]][parent_pos[1]][0]

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
                            if v != 0 and self.dist_coords(p, v) < 0.0387:
                                return True
                
        return False


