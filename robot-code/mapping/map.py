import numpy as np
import math
import cv2

class Map:

    def __init__(self):
        self.resolution = 0.06
        self.size = [2,2]
        self.range_x = [-self.resolution, self.resolution]
        self.range_y = [-self.resolution, self.resolution]
        
        # map[x, y]
        self.map = np.empty(self.size, dtype=object)
        for x in range(self.size[0]):
            for y in range(self.size[1]):
                self.map[x, y] = [0]


        # 0: unknown | 1: lidar | 2: camera | 3: passed by
        self.seen_map = np.zeros(self.size, dtype=int)


    def expand(self, point):
        if point[0] < self.range_x[0]:
            dif_map_x = math.ceil((self.range_x[0] - point[0])/self.resolution)
            while dif_map_x:
                self.map = np.insert(self.map, 0, None, axis=0)
                for y in range(np.size(self.map, 1)): self.map[0, y] = [0]
                self.seen_map = np.insert(self.seen_map, 0, 0, axis=0)
                self.range_x[0] = self.range_x[0] - self.resolution
                dif_map_x = dif_map_x - 1

        if point[0] > self.range_x[1]: 
            dif_map_x = math.ceil((point[0] - self.range_x[1])/self.resolution)
            while dif_map_x:
                self.map = np.insert(self.map, np.size(self.map, 0), None, axis=0)
                for y in range(np.size(self.map, 1)): self.map[np.size(self.map, 0)-1, y] = [0]
                self.seen_map = np.insert(self.seen_map, np.size(self.seen_map, 0), 0, axis=0)
                self.range_x[1] = self.range_x[1] + self.resolution
                dif_map_x = dif_map_x - 1

        if point[1] < self.range_y[0]: 
            dif_map_y = math.ceil((self.range_y[0] - point[1])/self.resolution)
            while dif_map_y:
                self.map = np.insert(self.map, 0, None, axis=1)
                for x in range(np.size(self.map, 0)): self.map[x, 0] = [0]
                self.seen_map = np.insert(self.seen_map, 0, 0, axis=1)
                self.range_y[0] = self.range_y[0] - self.resolution
                dif_map_y = dif_map_y - 1

        if point[1] > self.range_y[1]: 
            dif_map_y = math.ceil((point[1] - self.range_y[1])/self.resolution)
            while dif_map_y:
                self.map = np.insert(self.map, np.size(self.map, 1), None, axis=1)
                for x in range(np.size(self.map, 0)): self.map[x, np.size(self.map, 1)-1] = [0]
                self.seen_map = np.insert(self.seen_map, np.size(self.seen_map, 1), 0, axis=1)
                self.range_y[1] = self.range_y[1] + self.resolution
                dif_map_y = dif_map_y - 1

        return 


    def real_to_map(self, real_point):

        return [math.floor((real_point[0]-self.range_x[0])/self.resolution),
                math.floor((real_point[1]-self.range_y[0])/self.resolution)]
        

    def map_to_real(self, map_point):
        
        return [self.range_x[0]+map_point[0]*self.resolution+self.resolution/2,
                self.range_y[0]+map_point[1]*self.resolution+self.resolution/2]


    def add_point(self, point):
        self.expand(point)
        self.seen(point)

        mapx, mapy = self.real_to_map(point)

        for v in self.map[mapx, mapy]:
            if v != 0 and self.dist_coords(point, v) < 0.02:
                return
        self.map[mapx, mapy].append(point)

        return
    

    def seen(self, point):
        mapx, mapy = self.real_to_map(point)
        if mapx >= 0 and mapy >= 0 and mapx < np.size(self.seen_map, 0) and mapy < np.size(self.seen_map, 1):
            if self.seen_map[mapx, mapy] == 0:
                self.seen_map[mapx, mapy] = 1

        return
    

    def explored(self, point):
        mapx, mapy = self.real_to_map(point)
        if mapx >= 0 and mapy >= 0 and mapx < np.size(self.seen_map, 0) and mapy < np.size(self.seen_map, 1):
            self.seen_map[mapx, mapy] = 3

        return
    
    
    def print(self):
        for y in range(np.size(self.map, 1)):
            for x in range(np.size(self.map, 0)):
                print(self.map[x, y], end=" ")
            print(" ")


    def print_tile_map(self):
        # qnt_pontos = 1 # Quantidade de pontos dentro de um tile para ser considerado parede
        # divide cada tile por 3x3
        
        # Tam do tile: 0.12
        mapa_imprime = np.empty([self.size[0]*4 +1, self.size[1]*4 +1], dtype=object)
        for y in range(np.size(self.map, 1)):
            for x in range(np.size(self.map, 0)):
                tile_walls = np.empty([3, 3], dtype=object)
                tile_range_x = [self.range_x[0] + 0.12*x, self.range_x[0] + 0.12*(x+1)]
                tile_range_y = [self.range_y[0] + 0.12*y, self.range_y[0] + 0.12*(y+1)]
                for [coordX, coordY] in map[x, y]:
                    coordX -= tile_range_x[0]
                    coordY -= tile_range_y[0]
                    tile_walls[math.floor(coordX/0.12*3), math.floor(coordY/0.12*3)] = 1
                # atualiza no mapa_imprime
                for aux_y in [0, 1, 2]: # pula a casa central
                    for aux_x in [0, 1, 2]:
                        if aux_x != 1 or aux_y != 1:
                            mapa_imprime[2*x + aux_x+1, 2*y + aux_y+1] = tile_walls[aux_x, aux_y]

        for i in range(np.size(mapa_imprime, 1)):
            print(mapa_imprime[i], end=" ")
            print(" ")


    def print_real(self):
        for y in range(np.size(self.map, 1)):
            for x in range(np.size(self.map, 0)):
                print(self.map_to_real([x, y]), end=" ")
            print(" ")

        return
    

    def print_seen(self):
        for y in range(np.size(self.seen_map, 1)):
            for x in range(np.size(self.seen_map, 0)):
                print(self.seen_map[x, y], end=" ")
            print(" ")
    

    def to_png(self):
        transpose = np.zeros([np.size(self.map, 1), np.size(self.map, 0)])

        for y in range(np.size(self.map, 1)):
            for x in range(np.size(self.map, 0)):
                if len(self.map[x, np.size(self.map, 1)-1-y]) > 3:
                    transpose[y, x] = 1

        s = 'map.png'
        M = abs(transpose)*255
        cv2.imwrite(s, M)

        return 
    

    def to_png_explored(self):
        transpose = np.zeros([np.size(self.seen_map, 1), np.size(self.seen_map, 0)])

        for y in range(np.size(self.seen_map, 1)):
            for x in range(np.size(self.seen_map, 0)):
                if self.seen_map[x, np.size(self.seen_map, 1)-1-y] == 3:
                    transpose[y, x] = 1

        s = 'explored.png'
        M = abs(transpose)*255
        cv2.imwrite(s, M)

        return 
    

    def dist_coords(self, a, b):
        dist = ((a[0]-b[0])**2 + (a[1]-b[1])**2)**0.5
        return dist