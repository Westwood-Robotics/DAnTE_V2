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
from Play.motor_controller import MotorController
from Settings.Robot import *
import pdb

MC = MotorController(DAnTE.baudrate, DAnTE.port)
m_id = 1
# ping = MC.pbm.ping(m_id)
# print(ping)
mode = MC.pbm.get_mode(m_id)
iq = MC.pbm.get_present_iq(m_id)
goal_iq = MC.pbm.get_goal_iq(m_id)
enable = MC.pbm.get_torque_enable(m_id)
# p_gain = MC.pbm.get_p_gain_force(1,2,3)
# d_gain = MC.pbm.get_d_gain_force(1,2,3)
# # limit = MC.pbm.get_limit_position_min(2)
print(mode)
print(iq)
print(goal_iq)
print(enable)
# print(pos)
# print(p_gain)
# print(d_gain)
# # print(limit)



