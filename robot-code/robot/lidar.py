import math
import numpy as np


class Lidar:

    def __init__(self, hardware, time_step, map, gps, gyro):
        self.hardware = hardware
        self.time_step = time_step
        self.map = map
        self.gps = gps
        self.gyro = gyro

        # Hardware
        self.lidar = self.hardware.getDevice("lidar")
        self.lidar.enable(self.time_step*5)
        self.lidar.enablePointCloud()

        # Variables 
        self.point_cloud = []


    def update(self):
        self.point_cloud = np.array(self.lidar.getLayerPointCloud(2)[0:512])

        for i in range(len(self.point_cloud)):
            point = self.point_cloud[i]

            coordX = self.gps.front[0] + math.cos(self.gyro.last) * (-point.x) - math.sin(self.gyro.last) * (-point.y)
            coordY = self.gps.front[1] + math.sin(self.gyro.last) * (-point.x) + math.cos(self.gyro.last) * (-point.y)

            coordX_d = self.gps.front[0] + math.cos(self.gyro.last)*(-self.point_cloud[(i+2) % 512].x) - math.sin(self.gyro.last) * (-self.point_cloud[(i+2) % 512].y)
            coordY_d = self.gps.front[1] + math.sin(self.gyro.last)*(-self.point_cloud[(i+2) % 512].x) + math.cos(self.gyro.last) * (-self.point_cloud[(i+2) % 512].y)
            coordX_e = self.gps.front[0] + math.cos(self.gyro.last)*(-self.point_cloud[(i-2+512) % 512].x) - math.sin(self.gyro.last) * (-self.point_cloud[(i-2+512) % 512].y)
            coordY_e = self.gps.front[1] + math.sin(self.gyro.last)*(-self.point_cloud[(i-2+512) % 512].x) + math.cos(self.gyro.last) * (-self.point_cloud[(i-2+512) % 512].y)


            dist_ant = self.dist_coords([coordX, coordY], [coordX_d, coordY_e])
            dist_prox = self.dist_coords([coordX, coordY], [coordX_e, coordY_e])
            dist_ap = self.dist_coords([coordX_e, coordY_e], [coordX_d, coordY_d])

            if self.dist_coords(self.gps.front, [coordX, coordY]) < 0.98:
                if (dist_ap < 0.04) and (dist_ant < 0.01) and (dist_prox < 0.01):
                    self.map.add_obstacle([coordX, coordY])
                    #for i in range(20): 
                    #    self.map.seen([(i*self.gps.front[0]+(19-i)*coordX)/19, (i*self.gps.front[1]+(19-i)*coordY)/19])
            #else:
                #print("ferrou", i)

        self.map.to_png()

        return
    

    def ray_dist(self, ray):
        self.lidar.point_cloud = np.array(self.lidar.getLayerPointCloud(2)[0:512])
        point = self.lidar.point_cloud[ray]

        coordX = self.gps.front[0] + math.cos(self.gyro.last) * (-point.x) - math.sin(self.gyro.last) * (-point.y)
        coordY = self.gps.front[1] + math.sin(self.gyro.last) * (-point.x) + math.cos(self.gyro.last) * (-point.y)

        return self.dist_coords(self.gps.last, [coordX, coordY])
    

    def ray_front_dist(self, ray):
        self.lidar.point_cloud = np.array(self.lidar.getLayerPointCloud(2)[0:512])
        point = self.lidar.point_cloud[ray]

        return ((point.x)**2+(point.y)**2)**0.5
    
    
    def dist_coords(self, a, b):
        dist = ((a[0]-b[0])**2 + (a[1]-b[1])**2)**0.5
        return dist