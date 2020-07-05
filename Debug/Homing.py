#!usr/bin/env python
__author__ 		= "Min Sung Ahn"
__email__ 		= "aminsung@gmail.com"
__copyright__ 	= "Copyright 2019 RoMeLa"
__date__ = "January 1, 1999"

__version__ 	= "0.0.1"
__status__ 		= "Prototype"

import os
import pdb
import time
import numpy as np

from pybear import Manager
from Settings.Macros_BRUCE import *

pdm_lleg = Manager.BEAR(port='COM3', baudrate=8000000)
n_samples = 10

def home_motor(motor_id):

    print("Homing Motor ID: {}".format(motor_id))
    pdm = pdm_lleg

    nominal = 0.0

    raw_encoder_reading = 0.0
    for _ in range(n_samples):
        raw_encoder_reading += pdm.get_present_position(motor_id)[0]/10.0
        time.sleep(0.01)
    
    current_offset = pdm.get_homing_offset(motor_id)[0]

    new_offset = int(nominal + current_offset - raw_encoder_reading)
    if new_offset > 2**17:
        new_offset -= 2**18
    if new_offset < -2**17:
        new_offset += 2**18

    if new_offset > 2.0*np.pi*RAD2ENC:
        new_offset -= 2.0*np.pi*RAD2ENC
    elif new_offset < -2.0*np.pi*RAD2ENC:
        new_offset += 2.0*np.pi*RAD2ENC

    diff_offset = new_offset - current_offset

    print("Current Offset: {}".format(current_offset))
    print("New Offset: {}".format(new_offset))
    print("Difference: {}".format(diff_offset))

    new_joint_position = (raw_encoder_reading + diff_offset)*ENC2RAD

    print("Joint Position w/ New Offset: {}".format(new_joint_position))

    confirm = input("Set new offset: (y/n) ")
    if confirm == 'y':
        pdm.set_homing_offset((motor_id, new_offset))
        pdm.save_config(motor_id)
        print("Homing success!")
    else:
        print("Homing canceled.")

if __name__ == '__main__':
    print("BRUCE homing utility")
    while True:
        print("Ctrl + C to exit.")
        m_id = int(input("Motor ID to home: "))
        home_motor(m_id)
        os.system('clear')