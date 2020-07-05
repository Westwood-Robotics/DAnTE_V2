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


class RobotController_test(object):

    def __init__(self, robot=None):
        if robot is None:
            print("Robot set to DAnTE by default")
            self.robot = RobotDataStructure("DAnTE", 8000000, "/dev/ttyUSB0", [THUMB, INDEX, INDEX_M])
        else:
            self.robot = robot
        self.MC = MotorController(self.robot.baudrate, self.robot.port)
        self.approach_speed = None
        self.approach_stiffness = None
        self.detect_current = None
        self.final_strength = None
        self.max_iq = None

        self.ascii_art = True
        self.welcome_msg()

        self.start_robot()

    def welcome_msg(self):
        if self.ascii_art:
            print("=========== DAnTE version 2.0.0 -- Last Updated 2020.06.24 ===")
            print("==============================================================")

    # ------------------------
    # INITIALIZATION FUNCTIONS
    # ------------------------
    # Functions for initialization of finger(s)
    # Read initials.txt and check settings
    # Move finger(s) through range of motion according to initials.txt and check for mobility and interference
    def start_robot(self):
        # Read initials, fact check and populate robot object
        init_data = read_initials()  # init_data = [['FINGER', motor_id, homing_offset, travel]...]
        error = 0b000  # 3 bit respectively for THUMB, INDEX, INDEX_M
        for idx, f in enumerate(self.robot.fingerlist):
            if f.name != init_data[idx][0]:
                print("init_data.name does not match for %s." % f.name)
                error = error | (1 << idx)
            elif f.motor_id != init_data[idx][1]:
                print("init_data.motor_id does not match for %s." % f.name)
                error = error | (1 << idx)
            else:
                f.homing_offset = init_data[idx][2]
                f.travel = init_data[idx][3]
        if error:
            print("Failed to start robot.")
        else:
            print("Welcome aboard, Captain.")

    def set_init_PID(self, m_id):
        # Set PID
        # Velocity
        self.MC.pbm.set_p_gain_velocity((m_id, VEL_P))
        self.MC.pbm.set_i_gain_velocity((m_id, VEL_I))
        self.MC.pbm.set_d_gain_velocity((m_id, VEL_D))
        # Position
        self.MC.pbm.set_p_gain_position((m_id, POS_P))
        self.MC.pbm.set_i_gain_position((m_id, POS_I))
        self.MC.pbm.set_d_gain_position((m_id, POS_D))

    def set_init_PID_all(self):
        # Set PID for all fingers
        # Position
        self.MC.pbm.set_p_gain_position((THUMB.motor_id, POS_P), (INDEX.motor_id, POS_P), (INDEX_M.motor_id, POS_P))
        self.MC.pbm.set_i_gain_position((THUMB.motor_id, POS_I), (INDEX.motor_id, POS_I), (INDEX_M.motor_id, POS_I))
        self.MC.pbm.set_d_gain_position((THUMB.motor_id, POS_D), (INDEX.motor_id, POS_D), (INDEX_M.motor_id, POS_D))

    def set_init_PID_index(self):
        # Set PID for both index fingers
        # # Velocity PID
        # self.MC.pbm.set_p_gain_velocity((INDEX.motor_id, VEL_P), (INDEX_M.motor_id, VEL_P))
        # self.MC.pbm.set_i_gain_velocity((INDEX.motor_id, VEL_I), (INDEX_M.motor_id, VEL_I))
        # self.MC.pbm.set_d_gain_velocity((INDEX.motor_id, VEL_D), (INDEX_M.motor_id, VEL_D))
        # Position PID
        self.MC.pbm.set_p_gain_position((INDEX.motor_id, POS_P), (INDEX_M.motor_id, POS_P))
        self.MC.pbm.set_i_gain_position((INDEX.motor_id, POS_I), (INDEX_M.motor_id, POS_I))
        self.MC.pbm.set_d_gain_position((INDEX.motor_id, POS_D), (INDEX_M.motor_id, POS_D))

    def set_init_limits(self, m_id):
        self.MC.pbm.set_limit_iq_max((m_id, IQ_MAX_INIT))
        self.MC.pbm.set_limit_velocity_max((m_id, VEL_MAX_INIT))

    def set_approach_stiffness(self, gesture, approach_stiffness):
        # Set the P,D gains for Direct Force mode according to approach_stiffness

        force_p = approach_stiffness
        force_d = 0.1 * force_p

        if gesture == 'I':
            # Leave THUMB along if in pinch mode
            # Change the force PID of INDEX fingers
            self.MC.pbm.set_p_gain_force((BEAR_INDEX, force_p), (BEAR_INDEX_M, force_p))
            self.MC.pbm.set_d_gain_force((BEAR_INDEX, force_d), (BEAR_INDEX_M, force_d))
        else:
            # Change the force PID of all fingers
            self.MC.pbm.set_p_gain_force((BEAR_THUMB, force_p), (BEAR_INDEX, force_p), (BEAR_INDEX_M, force_p))
            self.MC.pbm.set_d_gain_force((BEAR_THUMB, force_d), (BEAR_INDEX, force_d), (BEAR_INDEX_M, force_d))
        print("Approach Stiffness set.")

    def grab(self, gesture, mode, **options):
        print("Grabbing Started.")

        # Control grab motion of self.robot
        # Start with a new_gesture, tripod(Y), pinch(I) or parallel(P)
        # Specify a grab mode: (H)old or (G)rip
        # Optional kwargs: (if not specified, go with a default value)
        # - Approach speed (approach_speed)
        # - Approach stiffness (approach_stiffness)
        # - Detection current (detect_current)
        # - Grip force/Hold stiffness (final_strength)
        # - Maximum torque current (max_iq)

        error = 0  # 1 for timeout, 2 for user interruption, 3 for initialization, 9 for Invalid input

        # Check enable and enable system if not.
        enabled = self.MC.pbm.get_torque_enable(BEAR_THUMB)
        if not enabled[0][0]:
            # Any motor not enabled
            print("WARNING: System not fully enabled. Enabling now...")
            self.MC.torque_enable(BEAR_THUMB, 1)

        # 0. Prep
        # Check input
        if gesture not in ['Y', 'I', 'P']:
            print("Invalid gesture input.")
            error = 9
            return error
        elif mode not in ['H', 'G']:
            print("Invalid mode input.")
            error = 9
            return error
        else:
            # Sort out all function input data.
            self.approach_speed = options.get("approach_speed", default_approach_speed)
            self.approach_stiffness = options.get("approach_stiffness", default_approach_stiffness)
            self.detect_current = options.get("detect_current", default_detect_current)
            self.final_strength = options.get("final_strength", default_final_strength)
            self.max_iq = options.get("max_iq", default_max_iq)

        print("Prep step finished.")

        # Calculate goal_approach_speed
        goal_approach_speed = [-self.approach_speed + 2 * self.approach_speed * f.mirrored for f in self.robot.fingerlist]

        print("Goal approach speed:")
        print(goal_approach_speed)

        # Set IQID and Velocity PID for all
        self.MC.init_driver_all()
        # Set IQ limit
        self.MC.pbm.set_limit_iq_max((BEAR_THUMB, self.max_iq), (BEAR_INDEX, self.max_iq), (BEAR_INDEX_M, self.max_iq))
        print("IQ_MAX set.")

        # 1. Start with change into gesture

        # 2. Set fingers' Direct Force PID according to Stiffness
        self.set_approach_stiffness(gesture, self.approach_stiffness)

        # Set into Direct Force Mode
        self.MC.set_mode_all('force')
        print("Mode set to Direct Force.")
        usr = input("Press enter to grab...")
        print("Approaching...")

        # 3. Fingers close tracking approach_speed, switch to final grip/hold upon object detection
        running = True
        # Python Version 3.7 or above
        # start_time = time.time_ns()/1000000000  # In ns unit
        # present_time = start_time
        start_time = time.time()
        present_time = start_time
        timeout = TIMEOUT_INIT * 2  # 2*TIMEOUT_INIT
        # velocity_window = [0 for i in range(MA_window)]
        # iq_window = [0 for i in range(MA_window)]
        # acceleration_window = [0 for i in range(MA_window)]

        velocity = 0
        velocity_error_int = 0
        iq = 0
        acceleration = 0

        velocity_log = []
        time_log = []
        iq_log = []
        iq_comp = []
        acceleration_log = []

        while not error and running:
            try:
                previous_time = present_time

                # Collect current status
                status = self.MC.get_present_status_all()
                # Get time
                present_time = time.time()
                delta_time = present_time - previous_time

                # Process data

                # Position
                position = status[0][0][0]

                # Velocity
                previous_vel = velocity
                # velocity_window.pop(0)
                # velocity_window.append(status[0][0][1])
                # velocity = sum(velocity_window) / MA_window
                velocity = SMOOTHING * status[0][0][1] + (1 - SMOOTHING) * velocity

                # iq
                # iq_window.pop(0)
                # iq_window.append(status[0][0][2])
                # iq = sum(iq_window) / MA_window
                iq = SMOOTHING * status[0][0][2] + (1 - SMOOTHING) * iq

                # Acceleration
                # acceleration_window.pop(0)
                # acceleration = acceleration_window.append((velocity - previous_vel) / delta_time)
                # acceleration = sum(acceleration_window) / (MA_window*20)
                acceleration = SMOOTHING * (velocity - previous_vel) / delta_time + (1 - SMOOTHING) * acceleration

                # Determine if contact
                # TODO: Experiment with it to find out the best detection method
                THUMB.contact = iq > self.detect_current

                # Check if all contact
                running = True

                # os.system('clear')
                print("Loop time:", delta_time)
                # print("Contact:")
                # print(THUMB.contact, INDEX.contact, INDEX_M.contact)
                # print("Running:", running)

                # Approach motion
                if running:
                    velocity_error = goal_approach_speed[0] - velocity
                    velocity_error_int += velocity_error * delta_time

                    approach_command = position + approach_p * velocity_error + approach_i * velocity_error_int - approach_d * acceleration

                    # Max approach_command
                    # approach_command = min(max(approach_command, -approach_command_max), approach_command_max)

                    # Send approach_command to BEARs
                    self.MC.pbm.set_goal_position((BEAR_THUMB, approach_command))

                    velocity_log.append(velocity)
                    iq_log.append(iq)
                    acceleration_log.append(acceleration*0.01)
                    time_log.append(present_time - start_time)
                    iq_comp.append(iq-acceleration*0.005)

                    # os.system('clear')
                    # print("Present Velocity:", velocity)
                    # print("Goal Velocity:", goal_approach_speed[0])
                    # print("Velocity Error:", velocity_error)
                    # print("Velocity Error Int:", velocity_error_int)
                    # print("Acceleration:", acceleration)
                    # print("Present Position", position)
                    # print("Approach Command:", approach_command)
                    # time.sleep(max(0.0, 0.006-delta_time))

                else:
                    print("Contact!")

            except KeyboardInterrupt:
                print("User interrupted.")
                running = False
                error = 2

        # Switch to final grip/hold upon object detection
        # TODO: Finish things here
        self.MC.torque_enable_all(0)
        plt.plot(time_log, iq_log, time_log, iq_comp, time_log, velocity_log)
        plt.show()

        return error


if __name__ == '__main__':
    rc = RobotController_test()

    rc.grab('P', 'H', approach_stiffness=1)
