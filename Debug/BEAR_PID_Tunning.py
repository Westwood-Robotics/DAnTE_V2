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
from Play.initialization import *
import pdb

MC = MotorController(DAnTE.baudrate, DAnTE.port)

# dbg = MC.pbm.bulk_read([BEAR_THUMB, BEAR_INDEX, BEAR_INDEX], ['present_position', 'present_velocity'])
# print(dbg)

# -------------------------
# Test to find out if motor automatically disable when mode changes.
# -------------------------
usr = input("Move THUMB to a safe position then press any key and enter.")
# Set PID and position mode, then get present position
MC.init_driver(THUMB.motor_id)
set_init_PID(THUMB.motor_id, MC)
MC.pbm.set_p_gain_force((THUMB.motor_id, 1.5))
MC.pbm.set_i_gain_force((THUMB.motor_id, 0))
MC.pbm.set_d_gain_force((THUMB.motor_id, 0.25))

MC.set_mode(THUMB.motor_id, 'position')
pos = MC.pbm.get_present_position(THUMB.motor_id)[0][0]
print(pos)
MC.pbm.set_limit_iq_max((THUMB.motor_id, 2.5))
# Enable then set goal pos to present pos
MC.torque_enable(THUMB.motor_id, 1)
MC.pbm.set_goal_position((THUMB.motor_id, pos))
mode = 2
running = True
while running:
    try:
        usr = input("Provide a new set of PD (g)ains (format: g P,D), "
                    "or change mode into (P)osition/(F)orce mode, or (E)xit:\n")
        if usr == 'P':
            MC.set_mode(THUMB.motor_id, 'position')
            mode = 2
            print("Changed to Position Mode.")
        elif usr == 'F':
            MC.set_mode(THUMB.motor_id, 'force')
            mode = 3
            print("Changed to Direct Force Mode.")
        elif usr[0] == 'g':
            gains_string = usr[2:].split(",")
            gains = [0, 0]
            gains[0] = float(gains_string[0])
            gains[1] = float(gains_string[1])
            if mode == 2:
                # MC.pbm.set_bulk_config((THUMB.motor_id, 'p_gain_position', gains[0],
                #                                         'd_gain_position', gains[1]))
                MC.pbm.set_p_gain_position((THUMB.motor_id, gains[0]))
                MC.pbm.set_d_gain_position((THUMB.motor_id, gains[1]))
            elif mode == 3:
                MC.pbm.set_bulk_config((THUMB.motor_id, 'p_gain_direct_force', gains[0],
                                                        'd_gain_direct_force', gains[1]))
            print("Changed PD gains to:", gains)
        elif usr == 'E':
            running = False
    except KeyboardInterrupt:
        print("User interrupted.")
        running = False
print("Exit...")
MC.torque_enable(THUMB.motor_id, 0)

