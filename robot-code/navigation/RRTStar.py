import math
import random
import numpy as np

class RRTStar:

    def __init__(self, map, pos):
        self.map = map 

        self.size = [2, 2]
        self.resolution = 0.06
        self.min_dist = 0.06
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


    def random_point(self):
        # Random point between map boundaries
        random.seed()
        x_percent = random.random()
        y_percent = random.random()
        x = self.range_x[0]*x_percent + self.range_x[1]*(1-x_percent)
        y = self.range_y[0]*y_percent + self.range_y[1]*(1-y_percent)
        return [x, y]
    

    def closest_point(self, point, op): 
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
                            if v != 0 and self.dist_coords(closest, point) > self.dist_coords(v[0], point) and (not op or not self.wall_between(v[0], point)) and v[3] == True:
                                closest = v[0]
            depth += 1
        if c == 50: print("WHILE CLOSEST")

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
    

    def update(self, pos, op):
        # Add current position to the Global RRT Graph 

        closest = self.closest_point(pos, 1)
        if op == 1: print("update closest", closest, self.dist_coords(pos, closest))
        if op == 1 or self.dist_coords(pos, closest) > self.min_dist:
            parent, self.cur_tile = self.add_graph(pos)
            if op == 1: print(parent)

        return
    

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
    

    def delete(self, pos):
        x, y = self.real_to_map(pos)

        print("delete", x, y)
        c = 0
        for v in self.graph[x, y]:
            print(v)
            if v != 0 and self.dist_coords(v[0], pos) < 0.01:
                # Change to False all child nodes too
                bfs = [[[x, y], c]]
                a = 0
                while len(bfs) > 0 and a < 100:
                    print("set false", bfs[0])
                    self.graph[bfs[0][0][0]][bfs[0][0][1]][bfs[0][1]][3] = False
                    print(self.graph[bfs[0][0][0]][bfs[0][0][1]][bfs[0][1]])
                    for b in self.graph[bfs[0][0][0]][bfs[0][0][1]][bfs[0][1]][2]: 
                        child = self.graph_child([[bfs[0][0][0], bfs[0][0][1]], bfs[0][1]], b)
                        if self.graph[child[0][0]][child[0][1]][child[1]][3] == True: bfs.append(child)
                    bfs.pop(0)
                    a += 1
                if a == 100: print("WHILE DELETE CHILD")
            c += 1

        return 


    def explore(self, ticks):
        # Generate and find unexplored nodes to the navigation

        unexplored = []
        self.graph_expand([self.map.range_x[0], self.map.range_y[0]])
        self.graph_expand([self.map.range_x[1], self.map.range_y[1]])

        while ticks > 0:
            ticks -= 1

            point = self.random_point()
            closest = self.closest_point(point, 0)
            point = self.project_point(point, closest)
            parent, pos = self.add_graph(point)
            if parent == [1000, 1000]: continue

            map_p = self.map.real_to_map(point)
            if map_p[0] >= 0 and map_p[0] < np.size(self.map.map, 0) and map_p[1] >= 0 and map_p[1] < np.size(self.map.map, 1):
                if self.map.seen_map[map_p[0], map_p[1]] == 0:
                    print("inexplorado", point, pos, parent)
                    unexplored.append(point) # [ponto (coordenada), posição]

        return unexplored
    

    def print(self):
        initial_x, initial_y = self.real_to_map(self.initial_center_pos)

        initial_node = self.graph[initial_x, initial_y][1]
        print("initial", initial_node)

        points = [[initial_node, [[initial_x, initial_y], 1]]]

        print("[", end = " ")
        while len(points) != 0:
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
        while pos != parent_pos and a < 500:
            s += self.dist_coords(point, parent_point)

            pos = parent_pos
            point = parent_point

            parent_pos = self.graph_parent(pos)
            parent_point = self.graph[parent_pos[0][0]][parent_pos[0][1]][parent_pos[1]][0]

            a += 1
        if a == 500: print("WHILE TOTAL_DIST")

        return s
    

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
                            if v != 0 and (self.dist_coords(p, v) < 0.036 or (self.dist_coords(p, self.initial_pos) > 0.005 and self.dist_coords(p, v) < 0.0376)):
                                return True
                
        return False


