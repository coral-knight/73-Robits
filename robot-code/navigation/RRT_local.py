import math
import random
import numpy as np
generator = np.random.default_rng()

class RRTLocal:

    def __init__(self, map, pos):
        self.map = map 

        self.size = [2, 2]
        self.resolution = 0.06
        self.min_dist = 0.04
        self.radius = 0.12
        self.range_x = [pos[0]-self.resolution, pos[0]+self.resolution]
        self.range_y = [pos[1]-self.resolution, pos[1]+self.resolution]

        # graph[x,y] = [ [ [i,j] , [ [x,y], 0], [[[x,y], z], [[x,y], z], ...]True], ...] -> [[coordenada], [[dist tiles pro pai], posição dentro do tile do pai], [filhos], útil]
        self.graph = np.empty(self.size, dtype=object)
        for x in range(self.size[0]):
            for y in range(self.size[1]):
                self.graph[x, y] = [0]

        print("creating RRT", self.real_to_map(pos), pos)
        self.graph[self.real_to_map(pos)[0], self.real_to_map(pos)[1]].append([pos, [[0, 0], 1], [], True])

        self.initial_pos = pos
        self.initial_center_pos = self.map_to_real(self.real_to_map(pos))
        self.cur_tile = [self.real_to_map(pos), 1]

        self.unexplored = []
        self.explore_bfs = [pos] # [[[x, y], c], [[x2, y2], c2]]


    def graph_expand(self, point):
        if point[0] < (self.range_x[0]+self.resolution):
            dif_map_x = math.ceil((self.range_x[0] - point[0])/self.resolution) + 1
            while dif_map_x:
                self.graph = np.insert(self.graph, 0, None, axis=0)
                for y in range(np.size(self.graph, 1)): self.graph[0, y] = [0]
                self.range_x[0] = self.range_x[0] - self.resolution
                self.cur_tile[0][0] += 1
                dif_map_x = dif_map_x - 1

        if point[0] > (self.range_x[1]-self.resolution): 
            dif_map_x = math.ceil((point[0] - self.range_x[1])/self.resolution) + 1
            while dif_map_x:
                self.graph = np.insert(self.graph, np.size(self.graph, 0), None, axis=0)
                for y in range(np.size(self.graph, 1)): self.graph[np.size(self.graph, 0)-1, y] = [0]
                self.range_x[1] = self.range_x[1] + self.resolution
                dif_map_x = dif_map_x - 1

        if point[1] < (self.range_y[0]+self.resolution): 
            dif_map_y = math.ceil((self.range_y[0] - point[1])/self.resolution) + 1
            while dif_map_y:
                self.graph = np.insert(self.graph, 0, None, axis=1)
                for x in range(np.size(self.graph, 0)): self.graph[x, 0] = [0]
                self.range_y[0] = self.range_y[0] - self.resolution
                self.cur_tile[0][1] += 1
                dif_map_y = dif_map_y - 1

        if point[1] > (self.range_y[1]-self.resolution): 
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
    

    def real_to_pos(self, real_point):

        [x, y] = self.real_to_map(real_point)
        
        print("pos", real_point, [x, y])

        closest = [[1000,1000], 0]
        dist = 1000

        for i in range(-1, 2):
            for j in range(-1, 2):
                if x+i >= 0 and x+i < np.size(self.graph, 0) and y+j >= 0 and y+j < np.size(self.graph, 1):
                    c = 0
                    for v in self.graph[x+i, y+j]:
                        if v != 0 and self.dist_coords(v[0], real_point) < dist: 
                            closest = [[x+i, y+j], c]
                            dist = self.dist_coords(v[0], real_point)
                        c += 1

        if dist < 0.1: return closest
        else: return [[1000, 1000], 1000]
        

    def map_to_real(self, map_point):
        
        return [self.range_x[0]+map_point[0]*self.resolution+self.resolution/2,
                self.range_y[0]+map_point[1]*self.resolution+self.resolution/2]


    def angle_point(self, gps, i):
        angle = (i * 0 + (47-i) * 2 * math.pi) / 47
        dist = 0.02

        x = gps[0] + dist * math.cos(angle)
        y = gps[1] + dist * math.sin(angle)

        return [x, y]


    def random_point(self, gps):
        angle_percent = generator.random()
        angle = 0 * angle_percent + 2 * math.pi * (1-angle_percent)

        dist_percent = generator.random()
        dist = 0.03 * dist_percent + 0.12 * (1-dist_percent)

        x = gps[0] + dist * math.cos(angle)
        y = gps[1] + dist * math.sin(angle)

        #print("local random point", [x, y], "from", gps)

        return [x, y]
    

    def closest_point(self, point, op): 
        # Find the closest node to the point

        mapx, mapy = self.real_to_map(point)

        closest = [[1000,1000], [[1000, 1000], 1000]]
        depth = 0

        while closest[0] == [1000,1000] and depth < 50:
            for y in range(mapy-depth, mapy+depth+1):
                for x in range(mapx-depth, mapx+depth+1, (1 if y == mapy-depth or y == mapy+depth else 2*depth)):
                    if x >= 0 and x < np.size(self.graph, 0) and y >= 0 and y < np.size(self.graph, 1):
                        c = 0
                        for v in self.graph[x, y]:
                            if v != 0 and (not op or not self.wall_between(v[0], point)) and v[3] == True:
                                if self.dist_coords(closest[0], point) > self.dist_coords(v[0], point) :
                                    closest[0] = v[0]
                                    closest[1] = [[x, y], c]
                            c += 1
            depth += 1
        if depth == 50: print("WHILE CLOSEST")

        return closest


    def add_graph(self, point):
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
                            #if v != 0 and v[0] == point: return [1000, 1000], [[0, 0], 0]
                            if v != 0 and self.dist_coords(v[0], point) <= self.radius and not self.wall_between(v[0], point) and v[3] == True:
                                if self.dist_coords(v[0], point) < 0.003: return [1000, 1000], [[0, 0], 0]
                                neighbours.append([v, cont])
                                if self.total_dist([[x, y], cont]) + self.dist_coords(v[0], point) < dist:
                                    parent = v[0]
                                    ppos = [[x, y], cont]
                                    pos = [[x-mapx, y-mapy], cont]
                                    dist = self.total_dist([[x, y], cont]) + self.dist_coords(v[0], point)
                            cont += 1
            depth += 1

        # Add 'point' to graph 
        if parent == [1000, 1000]: return [1000, 1000], [[0, 0], 0]
        self.graph[mapx, mapy].append([point, pos, [], True])
        self.graph[ppos[0][0], ppos[0][1]][ppos[1]][2].append([[mapx-ppos[0][0], mapy-ppos[0][1]], len(self.graph[mapx, mapy])-1])

        # Change parent for neighbours if passing through the new node results in a smaller path
        for v in neighbours:
            [p, c] = v
            if p[0] != parent and dist + self.dist_coords(point, p[0]) < self.total_dist([[mapx, mapy], len(self.graph[mapx, mapy])-1]):
                x, y = self.real_to_map(p[0])
                old = self.graph_parent([[x, y], c])
                self.graph[old[0][0]][old[0][1]][old[1]][2].remove([[x, y], c]) # Remove Child from the Parent of the neighbout
                self.graph[x][y][c][1] = [[mapx-x, mapy-y], len(self.graph[mapx, mapy])-1, False] # Change Parent of the neighbour
                self.graph[mapx][mapy][len(self.graph[mapx, mapy])-1][2].append([[x, y], c]) # Add Child to the point node

        return parent, [[mapx, mapy], len(self.graph[mapx, mapy])-1]


    def connect(self, pos, to):
        # Create a node to position 'pos' with parent on 'to' (normally 'pos' is the current position after finishing a "walk" action)
        if self.dist_coords(pos, to) < 0.003: return

        px, py = self.real_to_map(pos)
        tx, ty = self.real_to_map(to)

        print("connecting", pos, to)
        print("map", [px, py], [tx, ty])
        print("initial", self.cur_tile)

        parent = [[0, 0], 0]
        dist = 1000

        for i in range(-1, 2):
            for j in range(-1, 2):
                if tx+i >= 0 and tx+i < np.size(self.graph, 0) and ty+j >= 0 and ty+j < np.size(self.graph, 1):
                    tc = 0
                    for v in self.graph[tx+i, ty+j]:
                        if v != 0 and self.dist_coords(v[0], to) < dist:
                            dist = self.dist_coords(v[0], to)
                            parent = [[tx+i, ty+j], tc]
                        tc += 1

        if parent != [[0, 0], 0]:
            [[tx, ty], tc] = parent
            print("ligou em", parent, self.graph[tx, ty][tc][0])
            self.graph[px, py].append([pos, [[tx-px, ty-py], tc], [], True]) # Add new node
            self.graph[tx, ty][tc][2].append([[px-tx, py-ty], len(self.graph[px, py])-1]) # Add child
            self.cur_tile = [[px, py], len(self.graph[px, py])-1]

        '''if self.dist_coords([0, 0], to) < 0.002:
            print("ligamento inicial")
            self.graph[px, py].append([pos, [[self.cur_tile[0][0]-px, self.cur_tile[0][1]-py], self.cur_tile[1]], [], True])
            self.cur_tile = [[px, py], len(self.graph[px, py])-1]
        else:
            tc = 0
            for v in self.graph[tx, ty]:
                print(v)
                if v != 0 and self.dist_coords(v[0], to) < 0.007: 
                    print("ligou")
                    self.graph[px, py].append([pos, [[tx-px, ty-py], tc], [], True])
                    self.graph[tx, ty][tc][2].append([[px-tx, py-ty], len(self.graph[px, py])-1])
                    self.cur_tile = [[px, py], len(self.graph[px, py])-1]
                    break
                tc += 1'''

        return


    def explore(self, ticks):
        # Generate and find unexplored nodes to the navigation

        new = []

        while ticks > 0:
            ticks -= 1

            if len(self.explore_bfs) == 0: 
                print("explore bfs vazia")
                #self.print()
                return [[1000,1000]], self.unexplored

            cur = self.explore_bfs[0]
            self.explore_bfs.pop(0)

            #print("current", cur)

            for i in range(48):
                if i < 48: point = self.angle_point(cur, i)
                else: point = self.random_point(cur, i)

                p = self.real_to_map(point)
                g = self.graph[p[0], p[1]]

                if self.no_obstacle(point) and (len(g) <= 1 or all(self.dist_coords(point, u[0]) > 0.015 for u in g[1:])):
                    parent, _ = self.add_graph(point)
                    if parent == [1000, 1000]: continue

                    x, y = self.map.real_to_map(point)
                    information_gain = 0
                    if x >= 0 and x < np.size(self.map.map, 0) and y >= 0 and y < np.size(self.map.map, 1) and self.map.seen_map[x, y][0] != 3:
                        if self.map.seen_map[x, y][0] != 3: self.unexplored.append([point, information_gain]) 

                    new.append(point)
                    self.explore_bfs.append(point)

        return new, self.unexplored
    

    def print(self):
        initial_x, initial_y = self.real_to_map(self.initial_center_pos)

        initial_node = self.graph[initial_x, initial_y][1]
        print("initial", initial_node)

        points = [[initial_node, [[initial_x, initial_y], 1]]]

        print("[", end = " ")
        cont = 0
        while len(points) != 0 and cont < 4000:
            cont += 1
            node = points[0][0]
            pos = points[0][1]
            points.pop(0)

            for c in node[2]:
                child_pos = self.graph_child(pos, c)
                child = self.graph[child_pos[0][0]][child_pos[0][1]][child_pos[1]]
                if child[3] == False: continue
                points.append([child, [child_pos[0], child_pos[1]]])

                print("[", node[0], ",", child[0], "]", end = ", ")
        print("]", end = " ")

        if cont == 4000: print("WHILE PRINT RRT")


    '''========================================= AUXILIAR FUNCTIONS ==========================================='''


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
    

    def graph_child(self, pos, delta):
        # Get the position of a child of a graph point on the graph

        child_pos = [[0, 0], 0]
        child_pos[0][0] = pos[0][0] + delta[0][0]
        child_pos[0][1] = pos[0][1] + delta[0][1]
        child_pos[1] = delta[1]

        return child_pos


    def total_dist(self, pos):
        # Get the distance from the current node to the initial node

        s = 0

        point = self.graph[pos[0][0]][pos[0][1]][pos[1]][0]

        parent_pos = self.graph_parent(pos)
        parent_point = self.graph[parent_pos[0][0]][parent_pos[0][1]][parent_pos[1]][0]

        a = 0
        while pos != parent_pos and a < 2000:
            s += self.dist_coords(point, parent_point)

            pos = parent_pos
            point = parent_point

            parent_pos = self.graph_parent(pos)
            parent_point = self.graph[parent_pos[0][0]][parent_pos[0][1]][parent_pos[1]][0]

            a += 1
        if a == 2000: print("WHILE TOTAL_DIST")

        return s
    

    def no_obstacle(self, pos):
        map_p = self.map.real_to_map(pos)

        for x in range(-1, 2):
            for y in range(-1, 2):
                if map_p[0]+x >= 0 and map_p[1]+y >= 0 and map_p[0]+x < np.size(self.map.map, 0) and map_p[1]+y < np.size(self.map.map, 1):
                    for v in self.map.map[map_p[0]+x, map_p[1]+y]:
                        if v != 0 and self.dist_coords(pos, v) < 0.037:
                            return False
                        
        return True


    def wall_between(self, a, b):
        # Can the robot go safely from a to b?

        d = int(self.dist_coords(a, b)/0.02)
        if d == 0: d = 1
        
        for i in range(d+1): 
            p = [((d-i)*a[0]+i*b[0])/d, ((d-i)*a[1]+i*b[1])/d]
            map_p = self.map.real_to_map(p)

            for x in range(-1, 2):
                for y in range(-1, 2):
                    if map_p[0]+x >= 0 and map_p[1]+y >= 0 and map_p[0]+x < np.size(self.map.map, 0) and map_p[1]+y < np.size(self.map.map, 1):
                        for v in self.map.map[map_p[0]+x, map_p[1]+y]:
                            if v != 0 and self.dist_coords(p, v) < 0.037 and self.dist_coords(self.initial_pos, p) > 0.036:
                                return True
                
        return False


