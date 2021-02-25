#!usr/bin/env python
__author__ = "X. Zhang"
__email__ = "xzhang@westwoodrobotics.net"
__copyright__ = "Copyright 2020 Westwood Robotics Corporation"
__date__ = "Jan. 12, 2021"

__version__ = "0.0.1"
__status__ = "Prototype"

# This is a test script to perform forward kinematics calculation on the fingers before integrating into DT02.

from scipy.optimize import fsolve
import math as m
from Settings.Constants_DAnTE import *
import numpy as np
import pdb
from Settings.Robot import *

from mpl_toolkits import mplot3d
#matplotlib inline
import matplotlib.pyplot as plt


# Solve for gamma and delta
def angles(finger, palm_angle):

    # Get theta from palm_angle
    if finger.name == 'INDEX':
        i = 0
        theta = palm_angle - m.pi/2
    elif finger.name == 'INDEX_M':
        i = 1
        theta = m.pi/2 - palm_angle
    else:
        i = 2
        theta = 0
    alpha = finger.angles[0]
    beta = finger.angles[1]

    # 1. Solve for angle between middle phalanx and proximal phalanx, gamma
    # Use theta, alpha, beta to calculate omega for specified finger

    # Equation to solve takes the form of M*cos(beta+w) + N*sin(beta+w) + K = 0
    # Solve for beta+w then get w
    # Calculate constants for calculation:
    K = (l_2*l_2 + L_1*L_1 + l_1*l_1 + a[i]*a[i] + b[i]*b[i] + h*h - d*d)/2 \
        - b[i]*L_1*m.cos(beta)*m.cos(theta) - l_1*L_1*m.cos(alpha)*m.cos(beta)*m.sin(theta) \
        + a[i]*L_1*m.cos(beta)*m.sin(theta) - a[i]*l_1*m.cos(alpha) - l_1*L_1*m.sin(alpha)*m.sin(beta) \
        + h*L_1*m.sin(beta) - h*l_1*m.sin(alpha)
    M = l_2*(b[i]*m.cos(theta) + l_1*m.cos(alpha)*m.sin(theta) - a[i]*m.sin(theta) - L_1*m.cos(beta))
    N = l_2*(l_1*m.sin(alpha) - h - L_1*m.sin(beta))

    # Angle Solver using Math
    def func_1(x):
        # The function to pass to fsolve
        return M*m.cos(x) + N*m.sin(x) + K
    omega = fsolve(func_1, 3.14) - beta
    gamma = float(omega - m.pi/3)

    # 2. Solve for the angle between middle phalanx and distal phalanx, delta
    phi = 13/18*m.pi - gamma
    P = 2*l_3*l_4
    Q = -2*L_2*l_4
    S = l_3*l_3 + L_2*L_2 + l_4*l_4 - c*c - 2*l_3*L_2*m.cos(phi)

    # Angle Solver using Math
    def func_2(x):
        # The function to pass to fsolve
        return P * m.cos(x-phi) + Q * m.cos(x) + S
    tau = fsolve(func_2, 1)
    delta = float(tau - 55*m.pi/180)

    return [gamma, delta]


# Transfer matrix
def T(theta, alpha, a, d):
    # Generic transfer matrix
    T_i = np.array([[m.cos(theta),              -m.sin(theta),             0,             a],
                    [m.sin(theta)*m.cos(alpha), m.cos(theta)*m.cos(alpha), -m.sin(alpha), -m.sin(alpha)*d],
                    [m.sin(theta)*m.sin(alpha), m.cos(theta)*m.sin(alpha), -m.cos(alpha), -m.cos(alpha)*d],
                    [0,                         0,                         0,             1]])
    return T_i


# Finger Forward Kinematics, return joint locations
def finger_fk(finger, palm_angle):
    # Get theta_0 and theta_1 from palm_angle
    if finger.name == 'INDEX':
        theta_0 = -m.pi/6
        theta_1 = m.pi/6 + palm_angle - m.pi/2
    elif finger.name == 'INDEX_M':
        theta_0 = -m.pi*5/6
        theta_1 = -m.pi/6 + m.pi/2 - palm_angle
    else:
        theta_0 = m.pi/2
        theta_1 = 0

    # Calculate MCP joint location (and Proximal Phalanx direction)
    MCP = np.linalg.multi_dot([T(theta_0, 0, 0, 0), T(theta_1, 0, A, 0), T(finger.angles[1], m.pi/2, 0, 0)])
    # Calculate PIP joint location (and Middle Phalanx direction)
    PIP = np.dot(MCP, T(finger.angles[2], 0, L_1, 0))
    # Calculate DIP joint location (and Distal Phalanx direction)
    DIP = np.dot(PIP, T(finger.angles[3], 0, L_2, 0))
    # Calculate Tip location
    Tip = np.dot(DIP, T(0, 0, L_3, 0))

    # pdb.set_trace()

    # Create array joint_locations = [MCP; PIP; DIP; Tip]
    joint_locations = np.vstack((np.transpose(MCP[0:3, 3]),
                                 np.transpose(PIP[0:3, 3]),
                                 np.transpose(DIP[0:3, 3]),
                                 np.transpose(Tip[0:3, 3])))

    # TODO: add phalanx directions in output
    return joint_locations


def visual(robot=DAnTE):
    # Plot DT02
    fig = plt.figure()
    ax = plt.axes(projection='3d')
    outline_x = [-33.660, -44.919, -11.258, 11.258, 44.919, 33.660, -33.660]
    outline_y = [-32.434, -12.934, 45.368, 45.368, -12.934, -32.434, -32.434]
    outline_z = [0, 0, 0, 0, 0, 0, 0]
    ax.plot3D(outline_x, outline_y, outline_z, 'gray')  # Plot PALM outline
    ax.plot3D([0, 10], [0, 0], [0, 0], 'red')  # Plot x axis
    ax.plot3D([0, 0], [0, 10], [0, 0], 'green')  # Plot y axis
    ax.plot3D([0, 0], [0, 0], [0, 10], 'blue')  # Plot z axis

    loc_INDEX = np.transpose(robot.fingerlist[0].joint_locations)
    loc_INDEX_M = np.transpose(robot.fingerlist[1].joint_locations)
    loc_THUMB = np.transpose(robot.fingerlist[2].joint_locations)

    ax.plot3D(loc_INDEX[0, :], loc_INDEX[1, :], loc_INDEX[2, :], 'black')  # Plot INDEX
    ax.plot3D(loc_INDEX_M[0, :], loc_INDEX_M[1, :], loc_INDEX_M[2, :], 'black')  # Plot INDEX_M
    ax.plot3D(loc_THUMB[0, :], loc_THUMB[1, :], loc_THUMB[2, :], 'black')  # Plot THUMB

    plt.show()
