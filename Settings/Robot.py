#!usr/bin/env python
__author__ = "X. Zhang"
__email__ = "xzhang@westwoodrobotics.io"
__copyright__ = "Copyright 2020~2022 Westwood Robotics"
__date__ = "May 8, 2022"
__version__ = "0.1.3"
__status__ = "Beta"


from Settings.RobotData import *
from Settings.Macros_DAnTE import *
from Settings.Constants_DAnTE import *

THUMB = FingerDataStructure("THUMB", BEAR_THUMB, False, ENC_THUMB)
INDEX = FingerDataStructure("INDEX", BEAR_INDEX, False, ENC_INDEX)
INDEX_M = FingerDataStructure("INDEX_M", BEAR_INDEX_M, True, ENC_INDEX_M)
PALM = PalmDataStructure("PALM", DXL_PALM)
Empty_PALM = PalmDataStructure(name = "Empty_PALM", motor_id = 0)

# ---------
# Controller related settings
# ---------
PI0 = False # True if using pi0
BEAR_port = "COM9" # COM port for BEAR if not PI
DXL_port = "COM18" # COM port for DXL if not PI

try:
    with open('/sys/firmware/devicetree/base/model', 'r') as sysinfo:
        if 'raspberry pi zero' in sysinfo.read().lower():
            PI0 = True
except Exception:
    pass

if PI0:
    # Regular DAnTE:
    DAnTE = RobotDataStructure("DAnTE", "/dev/UB0010", 8000000, "/dev/WR-232H", 2000000, PALM, [INDEX, INDEX_M, THUMB])
    # 2-Finger DAnTE:
    DAnTE_2F = RobotDataStructure(name="DAnTE_2F",
                                  BEAR_port="/dev/UB0010",
                                  BEAR_baudrate=8000000,
                                  DXL_port=None,
                                  DXL_baudrate=0,
                                  palm=Empty_PALM,
                                  fingerlist=[INDEX, INDEX_M])
else:
    # Regular DAnTE:
    DAnTE = RobotDataStructure("DAnTE", BEAR_port, 8000000, DXL_port, 2000000, PALM, [INDEX, INDEX_M, THUMB])
    # 2-Finger DAnTE:
    DAnTE_2F = RobotDataStructure(name = "DAnTE_2F",
                                  BEAR_port = BEAR_port,
                                  BEAR_baudrate = 8000000,
                                  DXL_port = None,
                                  DXL_baudrate = 0,
                                  palm = Empty_PALM,
                                  fingerlist = [INDEX, INDEX_M])

