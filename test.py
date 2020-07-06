#!usr/bin/env python
__author__ = "Xiaoguang Zhang"
__email__ = "xzhang@westwoodrobotics.net"
__copyright__ = "Copyright 2020 Westwood Robotics"
__date__ = "Jan 8, 2020"
__version__ = "0.0.1"
__status__ = "Prototype"

from Settings.Robot import *
from Play.robot_controller import RobotController

rc = RobotController(robot=DAnTE, bypass_DXL=True)
rc.start_robot()
rc.initialization()
rc.grab('P', 'H', approach_speed=2, approach_stiffness=1, detect_current=0.3, max_iq=1, final_strength=3, logging=True)
