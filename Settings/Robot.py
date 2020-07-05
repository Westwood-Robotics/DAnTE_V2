#!usr/bin/env python
__author__ = "X. Zhang"
__email__ = "xzhang@westwoodrobotics.net"
__copyright__ = "Copyright 2020 Westwood Robotics Corporation"
__date__ = "Feb. 1, 2020"

__version__ = "0.0.1"
__status__ = "Prototype"


from Settings.RobotData import *

THUMB = FingerDataStructure("THUMB", BEAR_THUMB, False)
INDEX = FingerDataStructure("INDEX", BEAR_INDEX, False)
INDEX_M = FingerDataStructure("INDEX_M", BEAR_INDEX_M, True)

DAnTE = RobotDataStructure("DAnTE", "/dev/ttyUSB1", 8000000, "/dev/ttyUSB1", 2000000, [INDEX, INDEX_M, THUMB])
