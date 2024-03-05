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

    map.expand([0.09, 0.09])
    map.expand([-0.09, -0.09])
    map.print_map()

    map.add_point([0, 0], 2)
    map.add_point([0.03, 0.09], 1)

    map.print_real_map()

    map.map_png()

    #robot = Robot(time_step=16)
    #start = robot.hardware.getTime()
    #robot.run()
    
main()