import numpy
import math

class Map:

    def __init__(self):
        self.resolution = 0.06
        self.size = [2,2]
        self.range_x = [-self.resolution, self.resolution]
        self.range_y = [-self.resolution, self.resolution]
        
        # map[x, y]
        self.map = numpy.zeros(self.size)


    def expand(self, point):
        if point[0] < self.range_x[0]: 
            dif_map_x = math.ceil((self.range_x[0] - point[0])/self.resolution)

            while dif_map_x:
                map.insert(0, 0, axis=0)
                self.range_x[0] = self.range_x[0] - self.resolution
                dif_map_x = dif_map_x - 1

        if point[0] > self.range_x[1]: 
            dif_map_x = math.ceil((point[0] - self.range_x[1])/self.resolution)

            while dif_map_x:
                map.insert(map.size()-1, 0, axis=0)
                self.range_x[1] = self.range_x[1] - self.resolution
                dif_map_x = dif_map_x - 1

        if point[1] < self.range_y[0]: 
            dif_map_y = math.ceil((self.range_y[0] - point[1])/self.resolution)

            while dif_map_y:
                map.insert(0, 0, axis=1)
                self.range_y[0] = self.range_y[0] - self.resolution
                dif_map_y = dif_map_y - 1

        if point[1] > self.range_y[1]: 
            dif_map_y = math.ceil((point[1] - self.range_y[1])/self.resolution)

            while dif_map_y:
                map.insert(map[0].size()-1, 0, axis=1)
                self.range_y[1] = self.range_y[1] - self.resolution
                dif_map_y = dif_map_y - 1

        return 


    def real_to_map(self, real_point):
    
        return [math.floor((real_point[0]-self.range_x[0])/self.resolution),
                math.floor((real_point[1]-self.range_y[0])/self.resolution)]
        


    def map_to_real(self, map_point):

        return [self.range_x[0]+map_point[0]*self.resolution+self.resolution/2,
                 self.range_y[0]+map_point[1]*self.resolution+self.resolution/2]
           



    def add_point(self, point, type):
        return
        

    def print_map(self):
        for x in range(map.size()):
            for y in range(map[0].size()):
                print(map[x, y], end="")
            print(" ")
    
    #def map_jpeg