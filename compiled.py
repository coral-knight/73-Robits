#!/usr/bin/env python
import contextlib as __stickytape_contextlib

@__stickytape_contextlib.contextmanager
def __stickytape_temporary_dir():
    import tempfile
    import shutil
    dir_path = tempfile.mkdtemp()
    try:
        yield dir_path
    finally:
        shutil.rmtree(dir_path)

with __stickytape_temporary_dir() as __stickytape_working_dir:
    def __stickytape_write_module(path, contents):
        import os, os.path

        def make_package(path):
            parts = path.split("/")
            partial_path = __stickytape_working_dir
            for part in parts:
                partial_path = os.path.join(partial_path, part)
                if not os.path.exists(partial_path):
                    os.mkdir(partial_path)
                    with open(os.path.join(partial_path, "__init__.py"), "wb") as f:
                        f.write(b"\n")

        make_package(os.path.dirname(path))

        full_path = os.path.join(__stickytape_working_dir, path)
        with open(full_path, "wb") as module_file:
            module_file.write(contents)

    import sys as __stickytape_sys
    __stickytape_sys.path.insert(0, __stickytape_working_dir)

    __stickytape_write_module('robot/robot.py', b'from controller import Robot as Hardware\nfrom robot.process_sensors import Sensors\n\nclass Robot:\n    \'\'\'\n    General robot controller\n    \'\'\'\n    \n    def __init__(self, time_step):\n        self.time_step = time_step\n        \n        self.calibration_timer = 10\n        self.current_tick = 0\n\n        self.hardware = Hardware()\n        \n        self.sensors = Sensors(hardware=self.hardware, time_step=time_step)\n\n        #Left wheel\n        self.wheel_left = self.hardware.getDevice("wheel1 motor")\n        self.wheel_left.setPosition(float("inf"))\n\n        #Right wheel\n        self.wheel_right = self.hardware.getDevice("wheel1 motor")\n        self.wheel_right.setPosition(float("inf"))\n        \n        #Receiver\n        self.receiver = self.hardware.getDevice("receiver")\n        self.receiver.enable(time_step)\n\n        #Emmiter\n        self.emitter = self.hardware.getDevice("emitter")\n    \n    def speed(self, left_speed, right_speed):\n        \'\'\'\n        Sets the wheels rotationals speeds\n        \'\'\'\n        self.wheel_left.setVelocity(left_speed)\n        self.wheel_right.setVelocity(right_speed)\n        return\n\n    def send_victim(self, victim_position, victim_type):\n        \'\'\'\n        Sends victim information to server\n        \'\'\'\n        #message = struct.pack("i i c", int(victim_position[0], victim_position[1]), victim_type)\n        #self.emitter.send(message)\n        return\n\n    def send_endgame(self, map):\n        \'\'\'\n        Sends endgame message and map to server\n        \'\'\'\n        #message = struct.pack("i i c", int((self.final_coords[0]+initial_gps[0])*100), int((self.final_coords[1]+initial_gps[1])*100), victim_type)\n        #self.emitter.send(message)        \n        return \n\n    def run_calibration(self):\n        \'\'\'\n        Starts the simulation by calibrating the robot \n        \'\'\'\n        self.sensors.update_gps()\n        if self.current_tick < 5:\n            self.speed(2, 2)\n        elif self.current_tick == 5:\n            # gyro inicial\n            self.sensors.calibrate_gyro()\n        else:\n            self.speed(-2, -2)\n        return\n\n    def run_simulation(self):\n        \'\'\'\n        Runs a tick of the robot simulation\n        \'\'\'\n        return\n        \n\n    def run_endgame(self):\n        \'\'\'\n        Does the map cleanup and ends the simulation\n        \'\'\'\n        return\n        \n\n    def run(self):\n        \'\'\'\n        Starts the simulation by calibrating the robot and setting the current direction\n        \'\'\'\n        print("entrou aqui")\n        while self.hardware.step(self.time_step) != -1:\n            current_tick += 1\n            if self.current_tick <= self.calibration_timer:\n                self.run_calibration()\n            else:\n                self.run_simulation()\n            print("ultimo gps:", self.sensors.last_gps[0], self.sensors.last_gps[1])\n            print("ultimo front_gps:", self.sensors.front_gps[0], self.sensors.front_gps[1])\n            print("ultimo gyro:", self.sensors.last_gyro)\n        self.run_endgame()\n        return\n                ')
    __stickytape_write_module('robot/process_sensors.py', b'import math\n\nclass Sensors:\n    def __init__(self, hardware, time_step):\n        self.hardware = hardware\n        self.time_step = time_step\n\n        #GPS \n        self.gps = self.hardware.getDevice("gps")\n        self.gps.enable(time_step)\n        \'\'\'[x, z, -y]\'\'\'\n        self.initial_gps = self.gps.getValues()\n        self.last_gps = [0, 0]\n        self.front_gps = [0, 0]\n\n        #Gyroscope\n        self.gyro = self.hardware.getDevice("gyro")\n        self.gyro.enable(time_step)\n        self.last_gyro = 0 # nao e o inicial mesmo, precisa do run_calibration\n\n        #Centered camera\n        self.camera_front = self.hardware.getDevice("camera1")\n        self.camera_front.enable(time_step*5)\n        \n        #Increased central angle camera\n        self.camera_plus = self.hardware.getDevice("camera2")\n        self.camera_plus.enable(time_step*5)\n\n        #Decreased central angle camera\n        self.camera_sub = self.hardware.getDevice("camera3")\n        self.camera_sub.enable(time_step*5)\n\n        #LiDAR\n        self.lidar = self.hardware.getDevice("lidar")\n        self.lidar.enable(time_step*5)\n        self.lidar.enablePointCloud()\n\n\n    def update(self):\n        self.update_gps()\n        self.update_gyro()\n\n        # ve as cameras\n\n        # usa o LiDAR\n        return\n\n    def update_gps(self):\n        \'\'\' \n        Atualiza o GPS normalizado para o lado certo\n        \'\'\'\n        self.last_gps = [self.gps.getValues()[0] - self.initial_gps[0], -self.gps.getValues[2] + self.initial_gps[2]]\n        self.front_gps = [self.last_gps[0] + 0.03284 * math.cos(self.last_gyro), self.last_gps[1] + 0.03284 * math.sin(self.last_gyro)]\n        return \n\n    def update_gyro(self):\n        \'\'\' \n        Atualiza o Gyro normalizado\n        \'\'\'\n        self.last_gyro = self.last_gyro + self.gyro.getValues()[1]*self.time_step*0.001\n        if self.last_gyro > math.pi:\n            self.last_gyro -= 2*math.pi\n        if self.last_gyro < -math.pi:\n            self.last_gyro += 2*math.pi\n        return\n    \n    def calibrate_gyro(self):\n        \'\'\'\n        Sets the initial Gyro values\n        \'\'\'\n        self.last_gyro = math.atan2(self.last_gps[1] , self.last_gps[0])\n        return\n        ')
    # ====== =====    ====   ====  ====  = ===== =====
    #     =      =    =   = =    = =   = =   =   =
    #    =     ===    ====  =    = ====  =   =   =====
    #   =        =    =  =  =    = =   = =   =       =
    #  =     =====    =   =  ====  ====  =   =   =====
    
    from robot.robot import Robot
    
    def main():
        ''''''
        robot = Robot(time_step=16)
        robot.run()
        
    main()
