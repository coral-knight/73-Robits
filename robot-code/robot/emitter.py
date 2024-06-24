import math
import numpy as np
import struct

class Emitter:

    def __init__(self, hardware):
        self.hardware = hardware

        # Hardware
        self.emitter = self.hardware.getDevice("emitter")

        # Variables
        self.to_send = []


    def send(self, message):
        self.to_send.append(message)
        return


    def update(self):
        if len(self.to_send) > 0:
            self.emitter.send(self.to_send[0])
            self.to_send.pop(0)
            
        else: self.emitter.send(struct.pack('c', 'G'.encode(encoding="utf-8", errors="ignore")))

        return 
    
    
    