#!usr/bin/env python
__author__ = "Xiaoguang Zhang"
__email__ = "xzhang@westwoodrobotics.net"
__copyright__ = "Copyright 2020 Westwood Robotics"
__date__ = "Jan 8, 2020"
__version__ = "0.0.1"
__status__ = "Prototype"

from Settings.Robot import *
from Play.robot_controller import RobotController

rc = RobotController(robot=DAnTE)
rc.start_robot()
rc.initialization()
# # Release stand-alone test
# rc.set_robot_enable(0)
# print("Pose it.")
# usr = input("Press Enter to continue to release...")
# rc.release('F')

# # Change gesture stand-alone test
# usr = input("Press Enter to continue to change...")
# rc.change_gesture('I')

# grab(Y)-change_gesture(P)-grab(I)-release
usr = input("Press Enter to continue to grab-Y...")
rc.grab('Y', 'H', approach_speed=3)
usr = input("Press Enter to continue to change...")
rc.change_gesture('P')
usr = input("Press Enter to continue to grab-I...")
rc.grab('I', 'H')
usr = input("Press Enter to release-F...")
rc.release('F')


