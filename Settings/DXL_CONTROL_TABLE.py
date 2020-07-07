#!usr/bin/env python3.6
__author__ = "X Zhang"
__email__ = "xzhang@westwoodrobotics.net"
__copyright__ = "Copyright 2020 Westwood Robotics Corp."
__date__ = "Feb 14, 2020"

__version__ = "0.0.1"
__status__ = "Prototype"

'''
CONTROL_TABLE for Dynamixel X series actuator
'''
# Unit
POSITION_UNIT = 0.00153398078  # radian/value
VELOCITY_UNIT = 0.02398082392  # (radian/s)/value  0.229 rpm

# Control table address
# EEPROM
ADDR_X_MODE = 11
ADDR_X_DRIVE_MODE = 10
ADDR_X_BAUDRATE = 8
ADDR_X_HOMING_OFFSET = 20

# RAM
ADDR_X_TORQUE_ENABLE = 64  # Control table address is different in Dynamixel model
ADDR_X_GOAL_POSITION = 116
ADDR_X_PRESENT_POSITION = 132
ADDR_X_PRESENT_VELOCITY = 128
ADDR_X_PROFILE_VELOCITY = 112
ADDR_X_PROFILE_ACCELERATION = 108
ADDR_X_PRESENT_LOAD = 126
