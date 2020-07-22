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
# TODO: Release stand-alone test
rc.set_robot_enable(0)
print("Pose it.")
usr = input("Press Enter to continue to release...")
rc.release('F')

# TODO: change gesture stand-alone test
# TODO: initialize-grab(Y)-change_gesture(P)-grab(I)-release
