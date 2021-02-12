#!usr/bin/env python
__author__ 		= "Min Sung Ahn"
__email__ 		= "aminsung@gmail.com"
__copyright__ 	= "Copyright 2019 RoMeLa"
__date__ = "January 1, 1999"

__version__ 	= "1.0.0"
__status__ 		= "Prototype"

"""
Script that holds robot constants.
"""

# ==================================================
# ==================================================
# CONSTANTS
# ==================================================
# ==================================================
GRAV = 9.81
PI = 3.1415926535897932


# ==================================================
# ==================================================
# ROBOT INFO & KINEMATICS
# ==================================================
# ==================================================
# ------------------------------
# JOINT NAMES AND JOINT NO
# ------------------------------
# PALM = 0
# THUMB = 1
# INDEX = 2
# INDEX_M = 3

# ------------------------------
# GEOMETRY CONSTANTS [INDEX, INDEX_M, THUMB]
# ------------------------------
a = [0, 0, -4.5]  # Actuator location in y
b = [20.64, 20.64, 33.35]  # Actuator location in x
h = 93.5  # Actuator location in z
d = 113.4  # Linkage length
l_1 = 21.5  # Actuation Link length
l_2 = 10  # Middle Link drive arm length

L_1 = 25  # Proximal phalanx length
L_2 = 25  # Middle phalanx length
L_3 = 25  # Distal phalanx length

c = 27.1  # Coupling link length
l_3 = 6  # Proximal link drive arm length
l_4 = 6.5  # Distal link drive arm length

alpha_0 = [3.3746, 0.2330, 3.3561]  # alpha value [INDEX, INDEX_M, THUMB] when fingers homed and parallel (theta = 90)

# ==================================================
# ==================================================
# HARDWARE / SIMULATION SETTINGS
# ==================================================
# ==================================================
# ------------------------------
# MOTOR ID
# ------------------------------
BEAR_THUMB = 1
BEAR_INDEX = 2
BEAR_INDEX_M = 3

DXL_PALM = 1

# ------------------------------
# Pinging
# ------------------------------
PING_TRAIL_COUNT = 6

# ------------------------------
# BEAR MOTOR CHARACTERISTICS
# ------------------------------

# ------------------------------
# TORQUE LIMITS
# ------------------------------


# ------------------------------
# JOINT LIMITS [rad]
# ------------------------------

# ------------------------------
# MOTOR GAINS
# ------------------------------

# UNIVERSAL
IQID_P = 0.02
IQID_I = 0.02
IQID_D = 0

POS_P = 3
POS_I = 0.005
POS_D = 1.5

VEL_P = 0.4
VEL_I = 0.005
VEL_D = 0.0

FOR_P = 1.5
FOR_I = 0.0
FOR_D = 0.15

# IDLE
# TODO: experiment this
# These are position gains used for IDLE mode
IDLE_P = 3
IDLE_I = 0.002
IDLE_D = 2

# ------------------------------
# EXTERNAL ENCODERS
# ------------------------------
# We only have SPI bus 0 available to us on the Pi
BUS = 0
MAX_SPI_SPEED = 2000  # Hz
SPI_MODE = 0
# Below are encoder pins on Pi
ENC_THUMB = 22
ENC_INDEX = 27
ENC_INDEX_M = 17

# ==================================================
# ==================================================
# MOTION SETTINGS
# ==================================================
# ==================================================
# ------------------------------
# CALIBRATION
# ------------------------------
VEL_CAL = 1.5
# VEL_RESET = 5
IQ_CAL_DETECT = 0.65
VEL_CAL_DETECT = 0.005

# ------------------------------
# INITIALIZATION
# ------------------------------
VEL_MAX_INIT = 2.5
IQ_MAX_INIT = 1.5
TIMEOUT_INIT = 5

# ------------------------------
# GRAB
# ------------------------------
MA_window = 15  # Window for simple moving average
SMOOTHING = 0.07  # Smoothing factor for exponential moving average

ACC_COMP_FACTOR = 0.006

SPRING_COMP_START = 0.7
SPRING_COMP_BASE = 0.18
SPRING_COMP_FACTOR = 0.06

default_approach_speed = 1
default_approach_stiffness = 1
default_detect_current = 0.42
default_final_strength = 1
default_max_iq = 1.5

approach_speed_min = 0.5

# ------------------------------
# HOLD
# ------------------------------
HOLD_P_FACTOR = 1.5
HOLD_D_FACTOR = 1
default_hold_stiffness = 0.5

# ------------------------------
# GRIP
# ------------------------------



