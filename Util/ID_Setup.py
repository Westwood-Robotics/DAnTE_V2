#!usr/bin/env python
__author__ = "Xiaoguang Zhang"
__email__ = "xzhang@westwoodrobotics.net"
__copyright__ = "Copyright 2020 Westwood Robotics"
__date__ = "Jan 8, 2020"
__version__ = "0.0.1"
__status__ = "Prototype"

# -----------------------------
# Simple code to set ID for BEAR

from pybear import Manager
import time
import sys
import pdb

bear = Manager.BEAR(port="/dev/ttyUSB0", baudrate=8000000)
m_id = 1
m_id_new = int(input("Enter the new ID and press enter.\n"))
bear.set_id((m_id, m_id_new))
bear.save_config(m_id_new)
bear.save_config(m_id_new)
bear.save_config(m_id_new)