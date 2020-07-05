#!usr/bin/env python
__author__ = "X Zhang"
__email__ = "xzhang@westwoodrobotics.net"
__copyright__ = "Copyright 2020 Westwood Robotics Corp."
__date__ = "Feb 14, 2020"

__version__ = "0.0.1"
__status__ = "Prototype"


from Settings.Macros_DAnTE import *


class FingerDataStructure(object):

    def __init__(self, name, motor_id, mirrored):
        # Finger info
        self.name = name
        self.motor_id = motor_id
        self.mirrored = mirrored  # True of False, if the finger drive mechanism is mirrored from the thumb
        self.homing_offset = 0  # Homing_offset for BEAR so that when hand is in parallel the finger is fully open at 0
        self.travel = 0  # Total travel of BEAR for finger to close when hand is in parallel
        self.initialized = False  # Finger has been initialized
        self.contact = False  # Object contact status


class RobotDataStructure(object):
    def __init__(self, name, port, baudrate, DXL_port, DXL_baudrate, fingerlist=None):
        # Overall system info
        if fingerlist is None:
            fingerlist = [FingerDataStructure("THUMB", BEAR_THUMB, False)]
        self.name = name
        self.baudrate = baudrate
        self.port = port
        self.fingerlist = fingerlist
        self.finger_count = len(self.fingerlist)  # number of fingers
        self.finger_ids = []
        for f in fingerlist:
            self.finger_ids.append(f.motor_id)
        self.initialized = False  # Full hand has been initialized
        self.contact = False  # Object contact status
        self.booted = False

        self.DXL_baudrate = DXL_baudrate
        self.DXL_port = DXL_port


