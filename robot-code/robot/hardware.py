from controller import Robot as WebotsRobot


class Hardware:
    '''
    General robot hardware controller
    '''
    
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
    
    def speed(self, left_speed, right_speed):
        '''
        Sets the wheels rotationals speeds
        '''
        self.wheel_left.setVelocity(left_speed)
        self.wheel_right.setVelocity(right_speed)

    def send_victim(self, victim_position, victim_type):
        '''
        Sends victim information to server
        '''
        message = struct.pack("i i c", int(victim_position[0], victim_position[1]), victim_type)
        emitter.send(message)
        

    def send_endgame(self, map):
        '''
        Sends endgame message and map to server
        '''
        message = struct.pack("i i c", int((self.final_coords[0]+initial_gps[0])*100), int((self.final_coords[1]+initial_gps[1])*100), victim_type)
        emitter.send(message)
