#!usr/bin/env python
__author__ = "Xiaoguang Zhang"
__email__ = "xzhang@westwoodrobotics.net"
__copyright__ = "Copyright 2020 Westwood Robotics"
__date__ = "Jan 8, 2020"
__version__ = "0.1.0"
__status__ = "Beta"

# Prototype version
# Program to look for iq_comp for all fingers

from Settings.Robot import *
from Settings.Constants_DAnTE import *
from Play.robot_controller import RobotController
import numpy as np
import time
import matplotlib.pyplot as plt
from scipy.signal import butter, lfilter, freqz
import pdb


def butter_lowpass(cutoff, fs, order=5):
    nyq = 0.5 * fs
    normal_cutoff = cutoff / nyq
    b, a = butter(order, normal_cutoff, btype='low', analog=False)
    return b, a


def butter_lowpass_filter(data, cutoff, fs, order=5):
    b, a = butter_lowpass(cutoff, fs, order=order)
    y = lfilter(b, a, data)
    return y


rc = RobotController(robot=DAnTE)
rc.start_robot()
rc.initialization()

rc.change_gesture("P")

# Quasi-static position tracking to figure out iq comp for spring
rc.MC.set_mode_all('force')
rc.set_stiffness(3, 45, 1)

# Generate trajectory
finger_traj = []
sample_rate = 1000
for finger in rc.robot.fingerlist:
    finger_traj.append(np.linspace(0, finger.travel, sample_rate))

rc.set_robot_enable(1)
# Send all fingers to 0 position
rc.MC.pbm.bulk_write([BEAR_INDEX, BEAR_INDEX_M, BEAR_THUMB],
                     ['goal_position'],
                     [[finger_traj[0][99]], [finger_traj[1][99]], [finger_traj[2][99]]])
time.sleep(1)

# Now move along trajectory and collect data
data = []
t = []
finger_pos = [[],[],[]]
finger_velocity = [[],[],[]]
finger_iq = [[],[],[]]
start_time = time.time()
for i in range(100, 800):
    data.clear()
    data = rc.MC.pbm.bulk_read_write([BEAR_INDEX, BEAR_INDEX_M, BEAR_THUMB],
                                     ['present_position', 'present_velocity', 'present_iq'],
                                     ['goal_position'],
                                     [[finger_traj[0][i]], [finger_traj[1][i]], [finger_traj[2][i]]])
    t.append(time.time()-start_time)
    for j in range(3):
        finger_pos[j].append(data[j][0][0])
        finger_velocity[j].append(data[j][0][1])
        finger_iq[j].append(data[j][0][2])
    time.sleep(0.001)

# Get last data
data.clear()
data = rc.MC.pbm.bulk_read([BEAR_INDEX, BEAR_INDEX_M, BEAR_THUMB],
                           ['present_position', 'present_velocity', 'present_iq'])
t.append(time.time()-start_time)
for j in range(3):
    finger_pos[j].append(data[j][0][0])
    finger_velocity[j].append(data[j][0][1])
    finger_iq[j].append(data[j][0][2])

time.sleep(0.5)
rc.set_stiffness(3, 5, 1)
rc.release('F')
rc.set_robot_enable(0)

# convert position to position percentile
finger_pos_percent = []
for i in range(len(finger_pos)):
    finger_pos_percent.append((np.array(finger_pos[i])/rc.robot.fingerlist[i].travel).tolist())
#
# # Filter iq
# # Filter requirements.
# order = 6
# fs = sample_rate/4       # sample rate, Hz
# cutoff = 3  # desired cutoff frequency of the filter, Hz
#
# finger_iq_filtered = []
# # Filter the data, and plot both the original and filtered signals.
# for data in finger_iq:
#     filtered_data = butter_lowpass_filter(data, cutoff, fs, order)
#     finger_iq_filtered.append(filtered_data)
#
#
SMOOTHING_vel = 0.4
finger_velocity_filtered = [[0]*701, [0]*701, [0]*701]
for i in range(len(finger_velocity[0])):
    for j in range(3):
        if i == 0:
            finger_velocity_filtered[j][i] = finger_velocity[j][i]
        else:
            finger_velocity_filtered[j][i] = SMOOTHING_vel * finger_velocity[j][i] + \
                                             (1 - SMOOTHING_vel) * finger_velocity_filtered[j][i - 1]

SMOOTHING_iq = 0.2
finger_iq_filtered = [[0]*701, [0]*701, [0]*701]
for i in range(len(finger_velocity[0])):
    for j in range(3):
        if i == 0:
            finger_iq_filtered[j][i] = finger_iq[j][i]
        else:
            finger_iq_filtered[j][i] = SMOOTHING_iq * finger_iq[j][i] + \
                                             (1 - SMOOTHING_iq) * finger_iq_filtered[j][i - 1]
avrg_iq=[]
for data in finger_iq:
    avrg_iq.append(sum(data)/len(data))

print(avrg_iq)

# calculate contact possibility
possibility = [[],[],[]]
for i in range(len(finger_velocity[0])):
    for j in range(3):
        possibility[j].append(rc.contact_possibility(abs(finger_iq_filtered[j][i]),
                                                     abs(finger_velocity_filtered[j][i])))

# # Now plot position_percentile VS velocity
# plt.figure(1)
# plt.subplot(311)
# plt.plot(finger_pos_percent[0], finger_velocity[0])
# plt.subplot(312)
# plt.plot(finger_pos_percent[1], finger_velocity[1])
# plt.subplot(313)
# plt.plot(finger_pos_percent[2], finger_velocity[2])
# plt.show()
zero = [0]*(sample_rate+1)

plt.figure(1)
plt.subplot(311)
plt.plot(t, finger_velocity[0], 'b', t, finger_velocity_filtered[0], 'r', t, finger_iq_filtered[0], 'g', t,
         finger_iq[0], 'c', t, possibility[0], 'k')
plt.grid(True)
plt.subplot(312)
plt.plot(t, finger_velocity[1], 'b', t, finger_velocity_filtered[1], 'r', t, finger_iq_filtered[1], 'g', t,
         finger_iq[1], 'c', t, possibility[1], 'k')
plt.grid(True)
plt.subplot(313)
plt.plot(t, finger_velocity[2], 'b', t, finger_velocity_filtered[2], 'r', t, finger_iq_filtered[2], 'g', t,
         finger_iq[2], 'c', t, possibility[2], 'k')
plt.grid(True)
plt.show()



