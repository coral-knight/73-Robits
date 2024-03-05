import math
from robot.process_lidar import Lidar
from mapping.map import Map

class Sensors:
    def __init__(self, hardware, time_step):
        self.hardware = hardware
        self.time_step = time_step

        self.map = Map()

        #GPS 
        self.gps = self.hardware.getDevice("gps")
        self.gps.enable(self.time_step)
        '''[x, z, -y]'''
        self.initial_gps = [0, 0]
        self.last_gps = [0, 0]
        self.front_gps = [0, 0]

        #Gyroscope
        self.gyro = self.hardware.getDevice("gyro")
        self.gyro.enable(self.time_step)
        self.last_gyro = 0 

        #Centered camera
        self.camera_front = self.hardware.getDevice("camera1")
        self.camera_front.enable(self.time_step*5)
        
        #Increased central angle camera
        self.camera_plus = self.hardware.getDevice("camera2")
        self.camera_plus.enable(self.time_step*5)

        #Decreased central angle camera
        self.camera_sub = self.hardware.getDevice("camera3")
        self.camera_sub.enable(self.time_step*5)

        #LiDAR
        self.lidar = self.hardware.getDevice("lidar")
        self.lidar.enable(self.time_step*5)
        self.lidar.enablePointCloud()

        self.process_lidar = Lidar(self.lidar, self.map)


    def update(self, current_tick):
        self.update_gps()
        self.update_gyro()

        print(self.last_gyro)

        if current_tick % 5 == 0:
            self.process_lidar.update(self.front_gps, self.last_gyro)

        # ve as cameras

        return

    def update_gps(self):
        ''' 
        Atualiza o GPS normalizado para o lado certo
        '''
        self.last_gps = [self.gps.getValues()[0] - self.initial_gps[0], -self.gps.getValues()[2] + self.initial_gps[2]]
        self.front_gps = [self.last_gps[0] + 0.03284 * math.cos(self.last_gyro), self.last_gps[1] + 0.03284 * math.sin(self.last_gyro)]
        return 

    def update_gyro(self):
        ''' 
        Atualiza o Gyro normalizado
        '''
        self.last_gyro = self.last_gyro + self.gyro.getValues()[1]*self.time_step*0.001
        if self.last_gyro > math.pi:
            self.last_gyro -= 2*math.pi
        if self.last_gyro < -math.pi:
            self.last_gyro += 2*math.pi
        return
    
    def calibrate_gyro(self):
        '''
        Sets the initial Gyro values
        '''
        self.last_gyro = math.atan2(self.last_gps[1] , self.last_gps[0])
        return
        