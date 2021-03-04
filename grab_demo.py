#!usr/bin/env python
__author__ = "Xiaoguang Zhang"
__email__ = "xzhang@westwoodrobotics.net"
__copyright__ = "Copyright 2020 Westwood Robotics"
__date__ = "Jan 8, 2020"
__version__ = "0.1.0"
__status__ = "Beta"

from Settings.Robot import *
from Play.robot_controller import RobotController

rc = RobotController(robot=DAnTE)
rc.start_robot()
rc.initialization()


usr = input("Press Enter to continue to pinch sticky note.")
rc.grab('I', 'H', approach_speed=0.75, approach_stiffness=0.5, detect_current=0.2, final_strength=0.35)
usr = input("Press Enter to release...")
rc.release('F')

usr = input("Press Enter to continue to grab chip...")
rc.grab('Y', 'H', approach_speed=2.5, approach_stiffness=0.5, detect_current=0.35, final_strength=0.75)
usr = input("Press Enter to release...")
rc.release('F')


usr = input("Press Enter to continue to grab coroco...")
rc.grab('Y', 'G', approach_speed=3, approach_stiffness=0.5)
usr = input("Press Enter to release...")
rc.release('F')

usr = input("Press Enter to continue to grab-P...")
rc.grab('P', 'G', approach_speed=3, approach_stiffness=0.5, final_strength=1.5)
usr = input("Press Enter to release...")
rc.release('F')


rc.set_robot_enable(0)


