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
        self.point_cloud_1 = []
        self.point_cloud_3 = []


    def update(self):
        self.point_cloud = np.array(self.lidar.getLayerPointCloud(2)[0:512])
        self.point_cloud_1 = np.array(self.lidar.getLayerPointCloud(1)[0:512])
        self.point_cloud_3 = np.array(self.lidar.getLayerPointCloud(3)[0:512])

        for i in range(len(self.point_cloud)):
            # Normal Layer
            coordX = self.gps.front[0] + math.cos(self.gyro.last) * (-self.point_cloud[i].x) - math.sin(self.gyro.last) * (-self.point_cloud[i].y)
            coordY = self.gps.front[1] + math.sin(self.gyro.last) * (-self.point_cloud[i].x) + math.cos(self.gyro.last) * (-self.point_cloud[i].y)
            coordZ = self.gps.z + self.point_cloud[i].z

            # Layer 1
            coordX_1 = self.gps.front[0] + math.cos(self.gyro.last) * (-self.point_cloud_1[i].x) - math.sin(self.gyro.last) * (-self.point_cloud_1[i].y)
            coordY_1 = self.gps.front[1] + math.sin(self.gyro.last) * (-self.point_cloud_1[i].x) + math.cos(self.gyro.last) * (-self.point_cloud_1[i].y)
            coordZ_1 = self.gps.z + self.point_cloud_1[i].z

            # Layer 3 
            coordX_3 = self.gps.front[0] + math.cos(self.gyro.last) * (-self.point_cloud_3[i].x) - math.sin(self.gyro.last) * (-self.point_cloud_3[i].y)
            coordY_3 = self.gps.front[1] + math.sin(self.gyro.last) * (-self.point_cloud_3[i].x) + math.cos(self.gyro.last) * (-self.point_cloud_3[i].y)
            coordZ_3 = self.gps.z + self.point_cloud_3[i].z

            # Close points to get if it's not on the edge of the wall
            coordX_d = self.gps.front[0] + math.cos(self.gyro.last)*(-self.point_cloud[(i+2) % 512].x) - math.sin(self.gyro.last) * (-self.point_cloud[(i+2) % 512].y)
            coordY_d = self.gps.front[1] + math.sin(self.gyro.last)*(-self.point_cloud[(i+2) % 512].x) + math.cos(self.gyro.last) * (-self.point_cloud[(i+2) % 512].y)
            coordX_e = self.gps.front[0] + math.cos(self.gyro.last)*(-self.point_cloud[(i-2+512) % 512].x) - math.sin(self.gyro.last) * (-self.point_cloud[(i-2+512) % 512].y)
            coordY_e = self.gps.front[1] + math.sin(self.gyro.last)*(-self.point_cloud[(i-2+512) % 512].x) + math.cos(self.gyro.last) * (-self.point_cloud[(i-2+512) % 512].y)

            dist_ant = self.dist_coords([coordX, coordY], [coordX_d, coordY_e])
            dist_prox = self.dist_coords([coordX, coordY], [coordX_e, coordY_e])
            dist_ap = self.dist_coords([coordX_e, coordY_e], [coordX_d, coordY_d])

            if max(abs(coordX), abs(coordY)) < 0.98:
                if (dist_ap < 0.04) and (dist_ant < 0.01) and (dist_prox < 0.01):

                    # Get the bottom of the cone if it appears on 3 layers of ray
                    if (i > 384 or i < 128) and abs(coordZ_3 - (-0.066)) > 0.006 and max(abs(coordX_3), abs(coordY_3)) < 0.98:
                        if max(abs(coordX_1), abs(coordY_1)) < 0.98 and self.dist_coords([coordX, coordY], [coordX_3, coordY_3]) < 0.01:
                            ang_x_1, ang_y_1 = math.atan2(coordX_1-coordX, coordZ_1-coordZ), math.atan2(coordY_1-coordY, coordZ_1-coordZ)
                            ang_x_3, ang_y_3 = math.atan2(coordX-coordX_3, coordZ-coordZ_3), math.atan2(coordY-coordY_3, coordZ-coordZ_3)

                            if abs(ang_x_3 - ang_x_1) < 0.1 and abs(ang_y_3 - ang_y_1) < 0.1 and max(abs(ang_x_3), abs(ang_y_3)) > 0.15:
                                coordX_3 += math.tan(-ang_x_3) * (coordZ_3 + 0.066)
                                coordY_3 += math.tan(-ang_y_3) * (coordZ_3 + 0.066)
                                print("conical shaped", ang_x_3, ang_y_3)

                        self.map.add_obstacle([coordX_3, coordY_3])

                    self.map.add_obstacle([coordX, coordY])

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