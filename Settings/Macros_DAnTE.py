#!usr/bin/env python
__author__ 		= "Min Sung Ahn"
__email__ 		= "aminsung@gmail.com"
__copyright__ 	= "Copyright 2019 RoMeLa"
__date__ = "January 1, 1999"
__version__ = "0.1.0"
__status__ = "Beta"

"""
Script that holds macros for robot tuning.
"""

# ==================================================
# ==================================================
# MOTION SETTINGS
# ==================================================
# ==================================================

# ------------------------------
# GRAB Function
# ------------------------------

TIMEOUT_GRAB = 5

# detect_current smaller than detect_current_min will generate too much false detection thus not recommended.
# When using a detect_current smaller than confident_detect_current_min, detection must be confirmed for multiple times
# confident_detect_current_min needs no double check, slow or fast.
detect_current_min = 0.35
confident_detect_current = 0.4
detect_confirm = 2  # Times a detect must be confirmed before triggering

approach_i_limit = 1  # Limit I when approaching with velocity mode
approach_stiffness_min = 0.2
approach_d = 0  # Acceleration is too noisy, DO NOT USE

def approach_p_func(x):
    # approach p is a function of approach_stiffness
    # When approach stiffness is above 0.5, approach_p = 0.8
    # When approach stiffness is below 0.5, approach_p = 0.4/stiffness
    # Above is experiment result.
    if x < 0.5:
        return 0.4/x
    else:
        return 0.8

def approach_i_func(x):
    # approach i is a function of approach_stiffness
    # When approach stiffness is above 0.5, approach_i = 5
    # When approach stiffness is below 0.5, approach_i = 2.5/stiffness
    # Above is experiment result.
    if x < 0.5:
        return 2.5/x
    else:
        return 5


def approach_stiffness_func(speed, stiffness):
    # Function to determine approach stiffness
    # Stiffness can not be too small when speed is small
    return max(stiffness, 0.9/speed)


default_approach_speed = 1
default_approach_stiffness = 1
force_d_min = 0.05  # Minimum d gain for force mode to avoid oscillation
force_d_max = 1.5  # Maximum d gain for force mode to avoid noisy iq
default_detect_current = 0.42
default_preload = 0.5
default_max_iq = 3

# ------------------------------
# HOLD
# ------------------------------
delta_position = 0.15  # Move goal_position command inward by this amount at hold for firm contact
grip_confirm = 1000  # Number of times a grip must be confirmed before triggering, this is a rough method to filter
# out faulty grip confirmation

default_hold_stiffness = 0.5

# ------------------------------
# GRIP
# ------------------------------
TIMEOUT_GRIP = 3
grip_p = 2
grip_i = 12
grip_d = 0

# ------------------------------
# RELEASE
# ------------------------------
let_go_margin = 0.15  # Angular displacement to release in let-go mode
