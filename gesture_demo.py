#!usr/bin/env python
__author__ = "Xiaoguang Zhang"
__email__ = "xzhang@westwoodrobotics.net"
__copyright__ = "Copyright 2020 Westwood Robotics"
__date__ = "Jan 8, 2020"
__version__ = "0.1.0"
__status__ = "Beta"

# This is a demo program that goes through all gestures.

from Settings.Robot import *
from Play.robot_controller import RobotController
import time

rc = RobotController(robot=DAnTE)
rc.start_robot()
rc.initialization()

usr = input("Press Enter to start demo.")

rc.grab('Y', 'H', approach_speed=2)
time.sleep(0.5)
rc.grab('P', 'H', approach_speed=2)
time.sleep(0.5)
rc.grab('I', 'H', approach_speed=2)
time.sleep(0.5)
rc.change_gesture('Y')

rc.set_robot_enable(0)
