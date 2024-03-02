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

    map.expand([0.13, 0.13])
    map.expand([-0.13, -0.13])
    map.print_map()

    print(map.real_to_map([0, 0]))
    print(map.real_to_map([-0.10, 0.10]))
    map.add_point([0, 0], 1)
    map.add_point([-0.10, 0.10], 2)

    map.print_real_map()

    map.map_png()

    #robot = Robot(time_step=16)
    #start = robot.hardware.getTime()
    #robot.run()
    
main()