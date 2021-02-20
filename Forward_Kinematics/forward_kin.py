#!usr/bin/env python
__author__ = "X. Zhang"
__email__ = "xzhang@westwoodrobotics.net"
__copyright__ = "Copyright 2020 Westwood Robotics Corporation"
__date__ = "Jan. 12, 2021"

__version__ = "0.0.1"
__status__ = "Prototype"

# This is a test script to perform forward kinematics calculation on the fingers before integrating into DT02.

from scipy.optimize import fsolve
import math
from Settings.Constants_DAnTE import *


def solver(finger_name, palm_angle, alpha, beta):
    # Get theta from palm_angle
    if finger_name == 'INDEX':
        i = 0
        theta = palm_angle - math.pi/2
    elif finger_name == 'INDEX_M':
        i = 1
        theta = math.pi/2 - palm_angle
    else:
        i = 2
        theta = 0

    # 1. Solve for angle between middle phalanx and proximal phalanx, gamma
    # Use theta, alpha, beta to calculate omega for specified finger

    # Equation to solve takes the form of M*cos(beta+w) + N*sin(beta+w) + K = 0
    # Solve for beta+w then get w
    # Calculate constants for calculation:
    K = (l_2*l_2 + L_1*L_1 + l_1*l_1 + a[i]*a[i] + b[i]*b[i] + h*h - d*d)/2 \
        - b[i]*L_1*math.cos(beta)*math.cos(theta) - l_1*L_1*math.cos(alpha)*math.cos(beta)*math.sin(theta) \
        + a[i]*L_1*math.cos(beta)*math.sin(theta) - a[i]*l_1*math.cos(alpha) - l_1*L_1*math.sin(alpha)*math.sin(beta) \
        + h*L_1*math.sin(beta) - h*l_1*math.sin(alpha)
    M = l_2*(b[i]*math.cos(theta) + l_1*math.cos(alpha)*math.sin(theta) - a[i]*math.sin(theta) - L_1*math.cos(beta))
    N = l_2*(l_1*math.sin(alpha) - h - L_1*math.sin(beta))

    # Angle Solver using Math
    def func_1(x):
        # The function to pass to fsolve
        return M*math.cos(x) + N*math.sin(x) + K
    omega = fsolve(func_1, 3.14) - beta
    gamma = omega - math.pi/3

    # 2. Solve for the angle between middle phalanx and distal phalanx, delta
    phi = math.pi - gamma
    P = 2*l_3*l_4
    Q = -2*L_2*l_4
    S = l_3*l_3 + L_2*L_2 + l_4*l_4 - c*c - 2*l_3*l_4*math.cos(phi)

    # Angle Solver using Math
    def func_2(x):
        # The function to pass to fsolve
        return P * math.cos(x-phi) + Q * math.cos(x) + S
    tau = fsolve(func_2, 1)
    delta = tau - 55*math.pi/180

    return [gamma, delta]

