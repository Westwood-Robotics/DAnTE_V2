#!usr/bin/env python
__author__ = "X. Zhang"
__email__ = "xzhang@westwoodrobotics.io"
__copyright__ = "Copyright 2020~2022 Westwood Robotics"
__date__ = "May 8, 2022"
__version__ = "0.1.3"
__status__ = "Beta"

from Settings.Constants_DAnTE import *
import math
import numpy as np


class FingerDataStructure(object):

    def __init__(self, name, motor_id, mirrored, encoder=None):
        # Finger info
        self.name = name
        self.motor_id = motor_id
        self.mirrored = mirrored  # True of False, if the finger drive mechanism is mirrored from the thumb
        self.homing_offset = 0  # Homing_offset for BEAR so that when finger is fully open, BEAR is at 0
        self.travel = 0  # Total travel of BEAR for finger to close
        self.initialized = False  # Finger has been initialized
        self.contact = False  # Object contact status
        self.encoder = encoder  # Encoder pin# on Pi
        self.encoder_offset = -1  # Offset value for the encoder when finger is fully open
        self.angles = [0, 0, 0, 0]  # Phalanx angles, [alpha, beta, gamma, delta]
        self.joint_locations = np.zeros([4, 3])  # 3D joint_locations = [MCP; PIP; DIP; Tip]


class PalmDataStructure(object):

    def __init__(self, name, motor_id):
        # Palm info
        self.name = name
        self.motor_id = motor_id
        self.homing_offset = 0  # Homing_offset for Dynamixel, if any.
        self.travel = math.pi/2  # Total travel of BEAR for INDEX fingers to turn from parallel to pinch
        self.home = 0  # Home position for Dynamixel so that index fingers are in parallel
        self.initialized = False  # Palm has been initialized
        self.gesture = None  # Hand gesture status
        self.angle = 0  # theta
        # Sensors to be added.


class RobotDataStructure(object):
    def __init__(self, name, BEAR_port, BEAR_baudrate, DXL_port, DXL_baudrate, palm=None, fingerlist=None):
        # Overall system info

        self.name = name
        self.BEAR_baudrate = BEAR_baudrate
        self.BEAR_port = BEAR_port
        self.DXL_baudrate = DXL_baudrate
        self.DXL_port = DXL_port

        if fingerlist is None:  # If no finger specified, assume there is only a THUMB
            fingerlist = [FingerDataStructure("THUMB", BEAR_THUMB, False)]

        self.palm = palm
        self.fingerlist = fingerlist

        self.finger_count = len(self.fingerlist)  # number of fingers
        self.finger_ids = []
        self.encoders = []
        for f in fingerlist:
            self.finger_ids.append(f.motor_id)
            self.encoders.append(f.encoder)  # Create a cluster of encoder pins

        self.initialized = False  # Full hand has been initialized
        self.contact = False  # Object contact status
        self.booted = False  # System has booted
        self.iq_compensation = [0, 0, 0]  # iq compensation for more reliable contact possibility



