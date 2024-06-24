import math
import numpy as np
import struct

class Receiver:

    def __init__(self, hardware, time_step):
        self.hardware = hardware
        self.time_step = time_step

        # Hardware
        self.receiver = self.hardware.getDevice("receiver")
        self.receiver.enable(self.time_step)

        # Variables
        self.score = 0
        self.remaining_simulation_time = 10000
        self.remaining_real_time = 10000


    def update(self):
        if self.receiver.getQueueLength() > 0: 
            received_data = self.receiver.getBytes()
            data_len = len(received_data)

            if data_len == 16:
                tup = struct.unpack('c f i i', received_data)
                if tup[0].decode("utf-8") == 'G':
                    self.score = tup[1]
                    self.remaining_simulation_time = tup[2]
                    self.remaining_real_time = tup[3]
                    
            self.receiver.nextPacket()

        return 
    
    
    def calibrate(self):
        # Get initial GPS
        self.initial = self.gps.getValues()

        return