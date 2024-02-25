# ====== =====    ====   ====  ====  = ===== =====
#     =      =    =   = =    = =   = =   =   =
#    =     ===    ====  =    = ====  =   =   =====
#   =        =    =  =  =    = =   = =   =       =
#  =     =====    =   =  ====  ====  =   =   =====

from robot.robot import Robot

def main():
    ''''''
    
    robot = Robot(time_step=16)
    start = robot.hardware.getTime()
    robot.run()
    
main()