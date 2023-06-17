#!usr/bin/env python3
__author__ = "Xiaoguang Zhang"
__email__ = "xzhang@westwoodrobotics.net"
__copyright__ = "Copyright 2020 Westwood Robotics"
__date__ = "Jul 8, 2021"
__version__ = "0.1.0"
__status__ = "Beta"

# Run this script to perform a system demo. Choose options as prompted.

from Play.robot_controller import RobotController
from Settings.Robot import *
import time

run = False
rc = RobotController()
rc.start_robot()
if rc.initialization():
    run = True
else:
    print("Initialization failed, please debug. Exiting...")
    exit()

time.sleep(1)
rc.grab('G', gesture='P', approach_speed=0.5, preload=1, final_stiffness=1)
time.sleep(3)
rc.release()
