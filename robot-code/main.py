# ====== =====    ====   ====  ====  = ===== =====
#     =      =    =   = =    = =   = =   =   =
#    =     ===    ====  =    = ====  =   =   =====
#   =        =    =  =  =    = =   = =   =       =
#  =     =====    =   =  ====  ====  =   =   =====

from robot.robot import Robot
from mapping.map import Map

def main():
    ''''''
    map = Map()
    map.print_map()
    map.expand(0.13, 0.13)
    map.print_map()
    map.expand(-0.13, -0.13)
    map.print_map()
    #robot = Robot(time_step=16)
    #start = robot.hardware.getTime()
    #robot.run()
    
main()