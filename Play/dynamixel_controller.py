#!usr/bin/env python3.6
__author__ = "X Zhang"
__email__ = "xzhang@westwoodrobotics.net"
__copyright__ = "Copyright 2020 Westwood Robotics Corp."
__date__ = "Feb 14, 2020"

__version__ = "0.0.1"
__status__ = "Prototype"

'''
This module is used to communicate with the Dynamixel X series actuator on the DAnTE platform
'''

from dynamixel_sdk import *  # Uses Dynamixel SDK library
from Settings.DXL_CONTROL_TABLE import *
import math
import pdb


class DynamixelController(object):

    def __init__(self, DXL_ID, BAUDRATE=2000000, port='/dev/TTL-USB'):

        # Protocol version
        self.PROTOCOL_VERSION = 2.0  # See which protocol version is used in the Dynamixel

        # Default setting
        self.DXL_ID = DXL_ID  # Dynamixel ID : 1
        self.BAUDRATE = BAUDRATE  # Dynamixel default baudrate : 57600
        self.port = port

        self.port_handler = None
        self.packet_handler = None

        self.open_port()

    def open_port(self):

        # Initialize PortHandler instance
        # Set the port path
        # Get methods and members of PortHandlerLinux or PortHandlerWindows
        self.port_handler = PortHandler(self.port)

        # Initialize PacketHandler instance
        # Set the protocol version
        # Get methods and members of Protocol1PacketHandler or Protocol2PacketHandler
        self.packet_handler = PacketHandler(self.PROTOCOL_VERSION)
        # Open port
        if self.port_handler.openPort():
            # Set port baudrate
            if self.port_handler.setBaudRate(self.BAUDRATE):
                return True
            else:
                pass
        else:
            print("Failed to open the Dyanmixel port")
            return False

    def close_port(self):
        # Close port
        self.port_handler.closePort()

    def ping(self):
        # Ping Dynamixel
        # Get Dynamixel model number
        model_number, comm_result, error = self.packet_handler.ping(self.port_handler, self.DXL_ID)
        if comm_result != COMM_SUCCESS:
            print("%s" % self.packet_handler.getTxRxResult(comm_result))
            return False
        elif error != 0:
            print("%s" % self.packet_handler.getRxPacketError(error))
            return False
        else:
            return model_number

    def set_mode(self, mode):
        # Set the operation mode
        if mode == 'position':
            m = 3
        elif mode == 'velocity':
            m = 1
        elif mode == 'extended position':
            m = 4
        elif mode == 'PWM':
            m = 16
        else:
            print('Invalid operation mode.')
            return False

        comm_result, error = self.packet_handler.write1ByteTxRx(self.port_handler, self.DXL_ID, ADDR_X_MODE, m)
        if comm_result != COMM_SUCCESS:
            print("%s" % self.packet_handler.getTxRxResult(comm_result))
            return False
        elif error != 0:
            print("%s" % self.packet_handler.getRxPacketError(error))
            return False
        else:
            return True

    def set_baudrate(self, val):
        # Set the communication baudrate
        if val == 0:
            print("Baudrate set to 9600.")
        elif val == 1:
            print("Baudrate set to 57600.")  # Dynamixel default
        elif val == 2:
            print("Baudrate set to 115200.")
        elif val == 3:
            print("Baudrate set to 1000000.")
        elif val == 4:
            print("Baudrate set to 2000000.")
        else:
            print('Invalid baudrate selection.')
            return False

        comm_result, error = self.packet_handler.write1ByteTxRx(self.port_handler, self.DXL_ID, ADDR_X_BAUDRATE, val)
        if comm_result != COMM_SUCCESS:
            print("%s" % self.packet_handler.getTxRxResult(comm_result))
            return False
        elif error != 0:
            print("%s" % self.packet_handler.getRxPacketError(error))
            return False
        else:
            return True

    def set_drive_mode(self):
        # Set to velocity based profile, normal mode
        mode = 0
        comm_result, error = self.packet_handler.write1ByteTxRx(self.port_handler, self.DXL_ID, ADDR_X_DRIVE_MODE, mode)
        if comm_result != COMM_SUCCESS:
            print("%s" % self.packet_handler.getTxRxResult(comm_result))
            return False
        elif error != 0:
            print("%s" % self.packet_handler.getRxPacketError(error))
            return False
        else:
            return True

    def set_homing_offset(self, homing_offset):
        # Write homing offset, value in radian, -pi/2 ~ pi/2
        # Only worked in joint mode/basic position mode
        if abs(homing_offset) > math.pi/2:
            print("Input out of range.")
            return False
        homing_offset = int(homing_offset/POSITION_UNIT)
        comm_result, error = self.packet_handler.write4ByteTxRx(self.port_handler, self.DXL_ID, ADDR_X_HOMING_OFFSET, homing_offset)
        if comm_result != COMM_SUCCESS:
            print("%s" % self.packet_handler.getTxRxResult(comm_result))
            return False
        elif error != 0:
            print("%s" % self.packet_handler.getRxPacketError(error))
            return False
        else:
            return True

    def get_homing_offset(self):
        # Read present position, value in radian
        homing_offset, comm_result, error = self.packet_handler.read4ByteTxRx(self.port_handler, self.DXL_ID, ADDR_X_HOMING_OFFSET)
        if comm_result != COMM_SUCCESS:
            print("%s" % self.packet_handler.getTxRxResult(comm_result))
            return False
        elif error != 0:
            print("%s" % self.packet_handler.getRxPacketError(error))
            return False
        else:
            if homing_offset > 1024:
                # Negative homing offset
                homing_offset = homing_offset - 4294967296
            homing_offset = homing_offset*POSITION_UNIT
            return homing_offset

    def torque_enable(self, val):
        # Enable/dis-enable Dynamixel Torque
        comm_result, error = self.packet_handler.write1ByteTxRx(self.port_handler, self.DXL_ID, ADDR_X_TORQUE_ENABLE, val)
        if comm_result != COMM_SUCCESS:
            print("%s" % self.packet_handler.getTxRxResult(comm_result))
            return False
        elif error != 0:
            print("%s" % self.packet_handler.getRxPacketError(error))
            return False
        else:
            return True

    def get_enable(self):
        # Get torque enable status of Dynamixel
        val, comm_result, error = self.packet_handler.read1ByteTxRx(self.port_handler, self.DXL_ID, ADDR_X_TORQUE_ENABLE)
        if comm_result != COMM_SUCCESS:
            print("%s" % self.packet_handler.getTxRxResult(comm_result))
            return False
        elif error != 0:
            print("%s" % self.packet_handler.getRxPacketError(error))
            return False
        else:
            return bool(val)

    def set_goal_position(self, goal_position):
        # Write goal position, value in radian
        goal_position = int(goal_position/POSITION_UNIT)
        comm_result, error = self.packet_handler.write4ByteTxRx(self.port_handler, self.DXL_ID, ADDR_X_GOAL_POSITION, goal_position)
        if comm_result != COMM_SUCCESS:
            print("%s" % self.packet_handler.getTxRxResult(comm_result))
            return False
        elif error != 0:
            print("%s" % self.packet_handler.getRxPacketError(error))
            return False
        else:
            return True

    def get_goal_position(self):
        # Read present position, result in radian
        goal_position, comm_result, error = self.packet_handler.read4ByteTxRx(self.port_handler, self.DXL_ID, ADDR_X_GOAL_POSITION)
        if comm_result != COMM_SUCCESS:
            print("%s" % self.packet_handler.getTxRxResult(comm_result))
            return False
        elif error != 0:
            print("%s" % self.packet_handler.getRxPacketError(error))
            return False
        else:
            goal_position = goal_position*POSITION_UNIT
            return goal_position

    def get_present_position(self):
        # Read present position, result in radian
        present_position, comm_result, error = self.packet_handler.read4ByteTxRx(self.port_handler, self.DXL_ID, ADDR_X_PRESENT_POSITION)
        if comm_result != COMM_SUCCESS:
            print("%s" % self.packet_handler.getTxRxResult(comm_result))
            return False
        elif error != 0:
            print("%s" % self.packet_handler.getRxPacketError(error))
            return False
        else:
            present_position = present_position*POSITION_UNIT
            return present_position

    def get_present_velocity(self):
        # Read present position, result in radian
        present_velocity, comm_result, error = self.packet_handler.read4ByteTxRx(self.port_handler, self.DXL_ID, ADDR_X_PRESENT_VELOCITY)
        if comm_result != COMM_SUCCESS:
            print("%s" % self.packet_handler.getTxRxResult(comm_result))
            return False
        elif error != 0:
            print("%s" % self.packet_handler.getRxPacketError(error))
            return False
        else:
            present_velocity = present_velocity*VELOCITY_UNIT
            return present_velocity

    def set_profile_acceleration(self, val):
        """
        Sets acceleration of the Profile
        Unit: 214.577[rev / min2]
        Range: 0 ~ 32767    ‘0’ stands for an infinite acceleration

        :return:
        """
        comm_result, error = self.packet_handler.write4ByteTxRx(self.port_handler, self.DXL_ID, ADDR_X_PROFILE_ACCELERATION,
                                                                val)
        if comm_result != COMM_SUCCESS:
            print("%s" % self.packet_handler.getTxRxResult(comm_result))
            return False
        elif error != 0:
            print("%s" % self.packet_handler.getRxPacketError(error))
            return False
        else:
            return True

    def set_profile_velocity(self, val):
        """
        Sets maximum velocity of the Profile
        Unit: 0.229 [rev/min]
        Range: 0 ~ 32767    ‘0’ stands for an infinite velocity

        :return:
        """
        comm_result, error = self.packet_handler.write4ByteTxRx(self.port_handler, self.DXL_ID,
                                                                ADDR_X_PROFILE_VELOCITY,
                                                                val)
        if comm_result != COMM_SUCCESS:
            print("%s" % self.packet_handler.getTxRxResult(comm_result))
            return False
        elif error != 0:
            print("%s" % self.packet_handler.getRxPacketError(error))
            return False
        else:
            return True





