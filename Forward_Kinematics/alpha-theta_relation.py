#!usr/bin/env python
__author__ = "X. Zhang"
__email__ = "xzhang@westwoodrobotics.net"
__copyright__ = "Copyright 2020 Westwood Robotics Corporation"
__date__ = "Jan. 12, 2021"

__version__ = "0.0.1"
__status__ = "Prototype"

# The ranges of alpha for INDEX fingers change slightly when theta changes in DT02. This script is to explore such
# relationship between theta and alpha.

# INDEX finger is used in this script.

from scipy.optimize import fsolve
import math
from Settings.Constants_DAnTE import *
import numpy as np
import matplotlib.pyplot as plt


def get_constants(lp, bp, t):
    A = l_1*(a[0]+lp*math.cos(bp)*math.sin(t))
    B = l_1*(lp*math.sin(bp)+h)
    C = lp*(b[0]*math.cos(bp)*math.cos(t) - a[0]*math.cos(bp)*math.sin(t) - h*math.sin(bp))
    return A, B, C


def func(x, *constant):
    # The function to pass to fsolve
    A, B, C, D = constant
    return A*math.cos(x) + B*math.sin(x) + C + D


if __name__ == '__main__':
    # Get alpha when finger fully open
    L_p = 21.79  # L'
    beta_p = 36.59/180*math.pi  # beta'
    theta = np.linspace(-math.pi/2, 0, 100)
    theta = theta.tolist()
    alpha_open = []
    alpha_close = []
    for t in theta:
        A, B, C = get_constants(L_p, beta_p, t)
        D = (d * d - a[0] * a[0] - b[0] * b[0] - h * h - l_1 * l_1 - L_p * L_p) / 2
        constants = (A, B, C, D)
        alpha_open.append(float(fsolve(func, 3.4, args=constants))*180/math.pi)

    # Get alpha when finger fully close
    L_p = 33.7  # L'
    beta_p = 80.24 / 180 * math.pi  # beta'
    for t in theta:
        A, B, C = get_constants(L_p, beta_p, t)
        D = (d * d - a[0] * a[0] - b[0] * b[0] - h * h - l_1 * l_1 - L_p * L_p) / 2
        constants = (A, B, C, D)
        alpha_close.append(float(fsolve(func, 3.4, args=constants))*180/math.pi)

    alpha_d = [alpha_open[i]-alpha_close[i] for i in range(len(alpha_close))]

    plt.plot(theta, alpha_open, theta, alpha_close)
    plt.grid(True)
    plt.show()
    plt.plot(theta, alpha_d)
    plt.grid(True)
    plt.show()
