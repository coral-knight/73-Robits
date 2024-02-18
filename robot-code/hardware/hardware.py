from controller import Robot as WebotsRobot


class Hardware:
    '''General robot hardware'''
    
    def __init__(self, time_step):
        self.time_step = time_step
        
        self.robot = WebotsRobot()
        
        #Left wheel
        self.wheel_left = self.robot.getDevice("wheel1 motor")
        self.wheel_left.setPosition(float("inf"))

        #Right wheel
        self.wheel_right = self.robot.getDevice("wheel1 motor")
        self.wheel_right.setPosition(float("inf"))

        #GPS
        self.gps = self.robot.getDevice("gps")
        self.gps.enbale(time_step)

        #Gyroscope
        self.gyro = self.robot.getDevice("gyro")
        self.gyro.enable(time_step)

        #Centered camera
        self.camera_front = self.robot.getDevice("camera1")
        self.camera_front.enable(time_step*5)
        
        #Increased central angle camera
        self.camera_plus = self.robot.getDevice("camera2")
        self.camera_plus.enable(time_step*5)

        #Decreased central angle camera
        self.camera_sub = self.robot.getDevice("camera3")
        self.camera_sub.enable(time_step*5)
        
        #Receiver
        self.receiver = self.robot.getDevice("receiver")
        self.receiver.enable(time_step)

        #Emmiter
        self.emitter = self.robot.getDevice("emitter")

        #LiDAR
        self.lidar = self.robot.getDevice("lidar")
        self.lidar.enable(time_step*5)
        self.lidar.enablePointCloud()
    
    


