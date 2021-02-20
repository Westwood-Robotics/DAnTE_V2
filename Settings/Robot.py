#!usr/bin/env python
__author__ = "X. Zhang"
__email__ = "xzhang@westwoodrobotics.net"
__copyright__ = "Copyright 2020 Westwood Robotics Corporation"
__date__ = "Feb. 1, 2020"

__version__ = "0.0.1"
__status__ = "Prototype"


from Settings.RobotData import *
from Settings.Macros_DAnTE import *
from Settings.Constants_DAnTE import *

THUMB = FingerDataStructure("THUMB", BEAR_THUMB, False, ENC_THUMB)
INDEX = FingerDataStructure("INDEX", BEAR_INDEX, False, ENC_INDEX)
INDEX_M = FingerDataStructure("INDEX_M", BEAR_INDEX_M, True, ENC_INDEX_M)
PALM = PalmDataStructure("PALM", DXL_PALM)

DAnTE = RobotDataStructure("DAnTE", "/dev/UB000B", 8000000, "/dev/WR-232H", 2000000, PALM, [INDEX, INDEX_M, THUMB])
