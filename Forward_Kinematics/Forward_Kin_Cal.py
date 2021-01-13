#!usr/bin/env python
__author__ = "X. Zhang"
__email__ = "xzhang@westwoodrobotics.net"
__copyright__ = "Copyright 2020 Westwood Robotics Corporation"
__date__ = "Jan. 12, 2021"

__version__ = "0.0.1"
__status__ = "Prototype"

# This is a test script to perform forward kinematics calculation on the fingers before integrating into DT02.
# This test script is for THUMB. x-y plane is the palm plane, and the actuator output shaft is pointing to -x.

# Declare variables:
theta = 0  # Finger angle in x-y(palm) plane.
alpha = 0  # Angle between Actuation Link and y-asix. The Actuation Link is in the y-z plane
#  Variable to solve:
w = 0  # Angle of finger link. It is 0 when the link falls in x-y plane and pi/2 when coinside with z axis.
