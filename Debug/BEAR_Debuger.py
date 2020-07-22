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

MC = MotorController(DAnTE.BEAR_baudrate, DAnTE.BEAR_port)
m_id = 2
MC.pbm.set_bulk_config((m_id, 'p_gain_position', IDLE_P,
                              'i_gain_position', IDLE_I,
                              'd_gain_position', IDLE_D))
MC.set_mode(m_id, 'position')
MC.torque_enable(m_id, 1)
MC.pbm.set_goal_position((m_id, -0.5))




