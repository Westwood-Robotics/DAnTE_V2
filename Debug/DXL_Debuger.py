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


