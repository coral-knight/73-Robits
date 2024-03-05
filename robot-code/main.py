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

    map.add_point([0.03, 0.03])
    map.add_point([-0.03, -0.03])
    map.print_map()
    map.map_png()



    #robot = Robot(time_step=16)
    #start = robot.hardware.getTime()
    #robot.run()
    
main()