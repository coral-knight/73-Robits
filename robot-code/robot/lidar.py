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
        self.point_cloud = [[], [], [], []]

        self.last_update = [-1, -1, -1, -1]


    def update(self, current_tick):
        if self.last_update[1] < current_tick: self.point_cloud[1] = np.array(self.lidar.getLayerPointCloud(1)[0:512])
        if self.last_update[2] < current_tick: self.point_cloud[2] = np.array(self.lidar.getLayerPointCloud(2)[0:512])
        if self.last_update[3] < current_tick: self.point_cloud[3] = np.array(self.lidar.getLayerPointCloud(3)[0:512])

        self.last_update = [current_tick, current_tick, current_tick, current_tick]

        for i in range(len(self.point_cloud[2])):
            # Normal Layer
            [coordX, coordY] = self.ray_coords(i, 2, current_tick)
            coordZ = self.gps.z + self.point_cloud[2][i].z

            # Layer 1
            [coordX_1, coordY_1] = self.ray_coords(i, 1, current_tick)
            coordZ_1 = self.gps.z + self.point_cloud[1][i].z

            # Layer 3 
            [coordX_3, coordY_3] = self.ray_coords(i, 3, current_tick)
            coordZ_3 = self.gps.z + self.point_cloud[3][i].z

            # Close points to get if it's not on the edge of the wall
            [coordX_d, coordY_d] = self.ray_coords((i+1) % 512, 2, current_tick)
            [coordX_e, coordY_e] = self.ray_coords((i-1+512) % 512, 2, current_tick)

            [coordX_d_3, coordY_d_3] = self.ray_coords((i+1) % 512, 3, current_tick)
            [coordX_e_3, coordY_e_3] = self.ray_coords((i-1+512) % 512, 3, current_tick)

            # Dist to the close points
            dist_ant = self.dist_coords([coordX, coordY], [coordX_d, coordY_d])
            dist_prox = self.dist_coords([coordX, coordY], [coordX_e, coordY_e])
            dist_ap = self.dist_coords([coordX_e, coordY_e], [coordX_d, coordY_d])

            dist_ant_3 = self.dist_coords([coordX_3, coordY_3], [coordX_d_3, coordY_d_3])
            dist_prox_3 = self.dist_coords([coordX_3, coordY_3], [coordX_e_3, coordY_e_3])
            dist_ap_3 = self.dist_coords([coordX_e_3, coordY_e_3], [coordX_d_3, coordY_d_3])

            # Conditions to mark wall
            if self.dist_coords(self.gps.last, [coordX, coordY]) < 0.98:
                if (dist_ap < 0.02) and (dist_ant < 0.01) and (dist_prox < 0.01):

                    # Get the bottom of the cone if it appears on 3 layers of ray
                    if (i > 384 or i < 128) and abs(coordZ_3 - (-0.066)) > 0.006 and self.dist_coords(self.gps.last, [coordX_3, coordY_3]) < 0.3:
                        if (dist_ap_3 < 0.04) and (dist_ant_3 < 0.01) and (dist_prox_3 < 0.01):

                            if self.dist_coords(self.gps.last, [coordX_1, coordY_1]) < 0.98 and self.dist_coords([coordX, coordY], [coordX_3, coordY_3]) < 0.01:
                                ang_x_1, ang_y_1 = math.atan2(coordX_1-coordX, coordZ_1-coordZ), math.atan2(coordY_1-coordY, coordZ_1-coordZ)
                                ang_x_3, ang_y_3 = math.atan2(coordX-coordX_3, coordZ-coordZ_3), math.atan2(coordY-coordY_3, coordZ-coordZ_3)

                                if abs(ang_x_3 - ang_x_1) < 0.1 and abs(ang_y_3 - ang_y_1) < 0.1 and max(abs(ang_x_3), abs(ang_y_3)) > 0.15:
                                    coordX_3 += math.tan(-ang_x_3) * (coordZ_3 + 0.066)
                                    coordY_3 += math.tan(-ang_y_3) * (coordZ_3 + 0.066)
                                    #print("conical shaped", ang_x_3, ang_y_3)

                            self.map.add_obstacle([coordX_3, coordY_3])

                    self.map.add_obstacle([coordX, coordY])

        self.map.to_png()

        return
    

    def ray_coords(self, ray, layer, current_tick):
        if self.last_update[layer] < current_tick: self.point_cloud[layer] = np.array(self.lidar.getLayerPointCloud(layer)[0:512])
        self.last_update[layer] = current_tick

        point = self.point_cloud[layer][ray]

        coordX = self.gps.front[0] + math.cos(self.gyro.last) * (-point.x) - math.sin(self.gyro.last) * (-point.y)
        coordY = self.gps.front[1] + math.sin(self.gyro.last) * (-point.x) + math.cos(self.gyro.last) * (-point.y)

        return [coordX, coordY]


    def ray_dist(self, ray, current_tick):
        if self.last_update[2] < current_tick: self.point_cloud[2] = np.array(self.lidar.getLayerPointCloud(2)[0:512])
        self.last_update[2] = current_tick

        return self.dist_coords(self.gps.last, self.ray_coords(ray, 2, current_tick))
    

    def ray_front_dist(self, ray, current_tick):
        if self.last_update[2] < current_tick: self.point_cloud[2] = np.array(self.lidar.getLayerPointCloud(2)[0:512])
        self.last_update[2] = current_tick

        point = self.point_cloud[2][ray]

        return ((point.x)**2+(point.y)**2)**0.5
    
    
    def dist_coords(self, a, b):
        dist = ((a[0]-b[0])**2 + (a[1]-b[1])**2)**0.5
        return dist