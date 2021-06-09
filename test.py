#!usr/bin/env python
__author__ = "Xiaoguang Zhang"
__email__ = "xzhang@westwoodrobotics.net"
__copyright__ = "Copyright 2020 Westwood Robotics"
__date__ = "Jan 8, 2020"
__version__ = "0.1.0"
__status__ = "Beta"

from Play.robot_controller import RobotController

rc = RobotController()
rc.start_robot()
rc.initialization()
# Business Card Wide
rc.grab('I', 'G', approach_speed=3, preload=0.2, final_stiffness=1)
# Business Card Narrow
rc.grab('I', 'G', approach_speed=5, preload=0.2, final_stiffness=1)
# Folded Sticky Note or paper
rc.grab('I', 'G', approach_speed=1, preload=0.1, final_stiffness=0.5)

# Strawberry
rc.grab('Y', 'G', approach_speed=5, preload=0.1, final_stiffness=0.3)
# Cherry
rc.grab('I', 'G', approach_speed=2.5, preload=0.1, final_stiffness=0.2)
# Tomato
#TODO: Need soft phalanx
rc.grab('Y', 'G', approach_speed=3, preload=0.3, final_stiffness=0.2)
# Little Net Ladle
rc.grab('Y', 'G', approach_speed=5, preload=0.5, final_stiffness=0.7)
# Big net ladle
rc.grab('Y', 'H', approach_speed=3, preload=0.5, final_stiffness=1)


rc.grab('P', 'G', approach_speed=5, preload=3, final_stiffness=0.5)

input("Press Enter to release...")
rc.release('F')
rc.set_robot_enable(0)






