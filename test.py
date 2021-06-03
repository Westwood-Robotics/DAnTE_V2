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

input("Press Enter to grab...")
rc.grab('P', 'G')
rc.grab('P', 'G', approach_speed=0.5, preload=0.2)

input("Press Enter to release...")
rc.release('F')

rc.set_robot_enable(0)
