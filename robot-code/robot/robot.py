from hardware import Hardware
from controller import Robot as Hardware
from process_sensors import Sensors


class Robot:
    '''
    General robot controller
    '''
    
    def __init__(self, time_step, calibration_timer):
        self.time_step = time_step
        self.calibration_timer = calibration_timer
        
        self.hardware = Hardware()
        
        self.sensors = Sensors(self.hardware)

        #Left wheel
        self.wheel_left = self.hardware.getDevice("wheel1 motor")
        self.wheel_left.setPosition(float("inf"))

        #Right wheel
        self.wheel_right = self.hardware.getDevice("wheel1 motor")
        self.wheel_right.setPosition(float("inf"))

        #GPS
        self.gps = self.hardware.getDevice("gps")
        self.gps.enbale(time_step)

        #Gyroscope
        self.gyro = self.hardware.getDevice("gyro")
        self.gyro.enable(time_step)

        #Centered camera
        self.camera_front = self.hardware.getDevice("camera1")
        self.camera_front.enable(time_step*5)
        
        #Increased central angle camera
        self.camera_plus = self.hardware.getDevice("camera2")
        self.camera_plus.enable(time_step*5)

        #Decreased central angle camera
        self.camera_sub = self.hardware.getDevice("camera3")
        self.camera_sub.enable(time_step*5)
        
        #Receiver
        self.receiver = self.hardware.getDevice("receiver")
        self.receiver.enable(time_step)

        #Emmiter
        self.emitter = self.hardware.getDevice("emitter")

        #LiDAR
        self.lidar = self.hardware.getDevice("lidar")
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
        self.emitter.send(message)
        

    def send_endgame(self, map):
        '''
        Sends endgame message and map to server
        '''
        message = struct.pack("i i c", int((self.final_coords[0]+initial_gps[0])*100), int((self.final_coords[1]+initial_gps[1])*100), victim_type)
        self.emitter.send(message)        

    def run_calibration(self):
        '''
        Starts the simulation by calibrating the robot 
        '''

    def run_simulation(self):
        '''
        Runs a tick of the robot simulation
        '''

    def run_endgame(self):
        '''
        Does the map cleanup and ends the simulation
        '''
        

    def run(self):
        '''
        Starts the simulation by calibrating the robot and setting the current direction
        '''
        while self.hardware.step(self.time_step) != -1:
            if self.current_tick < self.calibration_timer:
                self.run_calibration()
            else:
                self.run_simulation()
        self.run_endgame()
        return
                