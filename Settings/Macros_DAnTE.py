#!usr/bin/env python
__author__ 		= "Min Sung Ahn"
__email__ 		= "aminsung@gmail.com"
__copyright__ 	= "Copyright 2019 RoMeLa"
__date__ = "January 1, 1999"

__version__ 	= "1.0.0"
__status__ 		= "Prototype"

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

TIMEOUT_GRAB = 3

# detect_current smaller than detect_current_min will generate too much false detection thus not recommended.
# When using a detect_current smaller than confident_detect_current_min, detection must be confirmed for multiple times
# confident_detect_current_min needs no double check, slow or fast.
detect_current_min = 0.35
confident_detect_current = 0.4
detect_confirm = 2  # Times a detect must be confirmed before triggering


approach_stiffness_min = 0.2
approach_p = 0.15
approach_d = 0  # Acceleration is too noisy, DO NOT USE


def approach_i_func(x):
    # approach i is a function of approach_stiffness
    # When approach stiffness is below 0.5, approach_i = 2.5
    # When approach stiffness is above 4.0, approach_i = 5
    # When in between, approach i is a function
    #        3         2
    # 0.119 x - 1.083 x + 3.417 x + 1.048
    # Above is experiment result.
    if x < 0.5:
        return 2.5
    elif x > 4.0:
        return 5
    else:
        return 0.119*x**3 - 1.083*x**2 + 3.417*x + 1.048


approach_command_max = 3

default_approach_speed = 1
default_approach_stiffness = 1
default_detect_current = 0.42
default_final_strength = 1
default_max_iq = 1.5

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
