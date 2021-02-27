#!usr/bin/env python
__author__ = "Xiaoguang Zhang"
__email__ = "xzhang@westwoodrobotics.net"
__copyright__ = "Copyright 2020 Westwood Robotics"
__date__ = "Feb 26, 2021"
__version__ = "0.1.0"
__status__ = "Beta"

# Change the direction of INDEX_M external encoder from default(CW) to CCW

from Play.MPS import MPS_Encoder
from Settings.Constants_DAnTE import *
from Settings.Robot import *

ext_enc = MPS_Encoder("MA310", BUS, INDEX_M.encoder, MAX_SPI_SPEED, SPI_MODE)

re = ext_enc.set_rotation_direction(1)
print(re)