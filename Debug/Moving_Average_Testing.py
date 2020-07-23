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
import matplotlib.pyplot as plt

import random

MC = MotorController(DAnTE.baudrate, DAnTE.port)
motor_id = 1
smoothing = 0.08  # 0~1, higher value discounts older observations faster.
window = 20
data_len = 100000
freq = 200
delta_t = 1/200
# Compare calculation speed of Simple Moving Average and Exponential moving Average on smoothing velocity
# Using THUMB, taking data and plot
velocity_window = []
SMA_history = []
EMA_history = []
velocity_history = []

SMA = 0  # Simple Moving Average
EMA = 0  # Exponential Moving Average
usr = input("Press enter to start.")

# Take initial data
for i in range(window):
    velocity = random.random()
    velocity_window.append(velocity)
    EMA = smoothing*velocity+(1-smoothing)*EMA
    time.sleep(delta_t)
SMA = sum(velocity_window)/window

# SMA
# Start the record:
start_time = time.time()
for i in range(data_len):
    velocity = random.random()
    velocity_window.pop(0)
    velocity_window.append(velocity)

    SMA = sum(velocity_window)/window

end_time = time.time()
print("Time took by SMA:", end_time-start_time)

# EMA
start_time = time.time()
for i in range(data_len):
    velocity = random.random()
    EMA = smoothing * velocity + (1 - smoothing) * EMA

end_time = time.time()
print("Time took by EMA:", end_time-start_time)

