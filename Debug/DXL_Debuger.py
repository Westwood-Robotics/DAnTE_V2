#!usr/bin/env python
__author__ = "Xiaoguang Zhang"
__email__ = "xzhang@westwoodrobotics.net"
__copyright__ = "Copyright 2020 Westwood Robotics"
__date__ = "Jan 8, 2020"
__version__ = "0.0.1"
__status__ = "Prototype"

import time
import os
import numpy as nmp
from Play.dynamixel_controller import DynamixelController
from Play.motor_controller import MotorController
from Settings.Robot import *
import pdb
robot = DAnTE

DC = DynamixelController(robot.palm.motor_id, robot.DXL_baudrate, robot.DXL_port)
data = DC.get_present_position()
print(data)
robot.palm.home = 1.38211668278
print(robot.palm.home)
time.sleep(0.5)
DC.torque_enable(1)
DC.set_goal_position(robot.palm.home)
user=input("Enter...")
DC.set_goal_position(robot.palm.home+math.pi/2*(15/16))
time.sleep(1)
data = DC.get_present_position()
print(data)
data = DC.get_goal_position()
print(data)



