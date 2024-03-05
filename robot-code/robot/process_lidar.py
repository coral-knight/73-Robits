import math
import numpy as np


class Lidar:

    def __init__(self, lidar, map):
        self.lidar = lidar
        self.map = map

    
    # arrumar contas (seno e cosseno)

    def update(self, front_gps, last_gyro):
        point_cloud = np.array(self.lidar.getLayerPointCloud(2)[0:512])

        for i in range(len(point_cloud)):
            point = point_cloud[i]

            coordX = front_gps[0] + math.cos(last_gyro) * (-point.x) - math.sin(last_gyro) * (-point.y)
            coordY = front_gps[1] + math.sin(last_gyro) * (-point.x) + math.cos(last_gyro) * (-point.y)

            coordX_d = front_gps[0] + math.cos(last_gyro)*(-point_cloud[(i+2) % 512].x) - math.sin(last_gyro) * (-point_cloud[(i+2) % 512].y)
            coordY_d = front_gps[1] + math.sin(last_gyro)*(-point_cloud[(i+2) % 512].x) + math.cos(last_gyro) * (-point_cloud[(i+2) % 512].y)
            coordX_e = front_gps[0] + math.cos(last_gyro)*(-point_cloud[(i-2+512) % 512].x) - math.sin(last_gyro) * (-point_cloud[(i-2+512) % 512].y)
            coordY_e = front_gps[1] + math.sin(last_gyro)*(-point_cloud[(i-2+512) % 512].x) + math.cos(last_gyro) * (-point_cloud[(i-2+512) % 512].y)


            dist_ant = self.dist_coords(coordX, coordY, coordX_d, coordY_d)
            dist_prox = self.dist_coords(coordX, coordY, coordX_e, coordY_e)
            dist_ap = self.dist_coords(coordX_e, coordY_e, coordX_d, coordY_d)


            if (dist_ap < 0.04) and (dist_ant < 0.01) and (dist_prox < 0.01):
                #print("add", i, coordX, coordY)
                self.map.add_point([coordX, coordY])

        self.map.to_png()

        return
    
    
    def dist_coords(self, a, b, x, y):
        dist = ((a-x)**2 + (b-y)**2)**0.5
        return dist