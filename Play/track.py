#!usr/bin/env python
__author__ = "Xiaoguang Zhang"
__email__ = "xzhang@westwoodrobotics.net"
__copyright__ = "Copyright 2020 Westwood Robotics"
__date__ = "Jan 8, 2020"
__version__ = "0.0.1"
__status__ = "Prototype"

from pybear import Manager
import time
import os

#-----------------------------
# Code for single finger position tracking under Direct Force mode
# This code will envetually be turned into a function as follows:
# track(motor_id, stiffness, current_limit, pos)

# Stay simple for now: no external PID loop, only rely on the BEAR Direct Force PID

# User set a stiffness, ranging from 1~10, and the stiffness value maps to a certain PID setting
# pos ranges from 0-1000, maps to the actual finger range.
motor_id =
stiffness =
current_limit =
pos =

# Pos ranges
pos_limit = 100000

# Calculate PID and goal pos
# When the stiffness is too low, engage external PID loop to reduce steady state error
pos_goal = pos_limit*pos/1000
p_force =
d_force =

# Send command to motor
