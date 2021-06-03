#!usr/bin/env python3
__author__ = "Xiaoguang Zhang"
__email__ = "xzhang@westwoodrobotics.net"
__copyright__ = "Copyright 2020 Westwood Robotics"
__date__ = "Jan 8, 2020"
__version__ = "0.1.0"
__status__ = "Beta"

import time
import os
import sys
from pathlib import Path
from Play.motor_controller import MotorController
from Play.dynamixel_controller import DynamixelController
from Settings.Robot import *
from Settings.Constants_DAnTE import *
import matplotlib.pyplot as plt
import matplotlib
matplotlib.interactive(True)
import math
from collections import deque

if EXTERNAL_ENC:
    # Only import the following when EXTERNAL_ENC is True as wiringpi and spidev are required.
    import Forward_Kinematics.forward_kin as FK
    from Play.MPS import MPS_Encoder_Cluster

import pdb

class RobotController(object):

    def __init__(self, robot=None, bypass_DXL=False, bypass_ext_enc=False):
        if robot is None:
            print("Robot set to DAnTE by default")
            robot = DAnTE

        self.robot = robot
        self.MC = MotorController(self.robot.BEAR_baudrate, self.robot.BEAR_port)

        # When debug, you might want to bypass Dynamixel
        self.bypass_DXL = bypass_DXL
        if self.bypass_DXL:
            self.DC = None
        else:
            self.DC = DynamixelController(self.robot.palm.motor_id, self.robot.DXL_port, self.robot.DXL_baudrate)

        if not EXTERNAL_ENC:
            # Force to bypass external encoders when EXTERNAL_ENC=None
            bypass_ext_enc = True

        # When debug, you might want to bypass external encoders
        self.bypass_ext_enc = bypass_ext_enc
        if self.bypass_ext_enc:
            self.ext_enc = None
        else:
            self.ext_enc = MPS_Encoder_Cluster("MA310", BUS, robot.encoders, MAX_SPI_SPEED, SPI_MODE)
            # self.robot.DXL_port)

        # self.gesture = None
        self.mode = None

        self.approach_speed = None
        self.approach_stiffness = None
        self.detect_current = None
        self.final_stiffness = None
        self.preload = None
        self.max_iq = None
        self.contact_position = [0, 0, 0]
        # self.contact_iq = [0, 0, 0]
        self.balance_factor = [1, 1, 1]  # Factor for force balance between fingers, update when change gesture

        self.welcome_msg()

        # self.start_robot()

    def welcome_msg(self):
        print("=========== DAnTE version 2.0.0 -- Last Updated 2020.06.24 ===")
        print("==============================================================")

    # ------------------------
    # INITIALIZATION / CALIBRATION
    # ------------------------
    # Functions for initialization of finger(s)
    # Read initials.txt and check settings
    # Move finger(s) through range of motion according to initials.txt and check for mobility and interference
    def start_robot(self):
        # Ping all actuators
        error = self.ping()  # 4 bit respectively for INDEX, INDEX_M, THUMB, Dynamixel, 0b10000 for overall error.

        # Read initials, fact check and populate robot object
        init_data = self.read_initials()  # init_data = [['FINGER', motor_id, homing_offset, travel, encoder_offset]...]
        if not init_data or (not self.bypass_DXL and len(init_data) < 4) or (self.bypass_DXL and len(init_data) < 3):
            print("Initialization data seems to be corrupted. Please run calibration_geometry() first.")
            error = 0b10000
            return error

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
                if not self.bypass_ext_enc:
                    f.encoder_offset = init_data[idx][4]
        if not self.bypass_DXL:
            if self.robot.palm.name != init_data[3][0]:
                print("init_data.name does not match for %s." % self.robot.palm.name)
                error = error | (1 << 3)
            elif self.robot.palm.motor_id != init_data[3][1]:
                print("init_data.motor_id does not match for %s." % self.robot.palm.name)
                error = error | (1 << 3)
            else:
                self.robot.palm.home = init_data[3][2]
                self.robot.palm.travel = init_data[3][3]

        if error:
            print("Failed to start robot.")
            sys.exit()

        else:
            # Set Current, Velocity and Position PID as well as safe iq_max and velocity_max,
            # and clear Direct Force PID.
            self.MC.init_driver_all()
            self.robot.booted = True
            print("Welcome aboard, Captain.")

        # return error

    def initialization(self):
        """Full hand initialization."""

        # Check if booted
        if self.robot.booted:
            pass
        else:
            print("Run start_robot first.")
            return False

        abnormal = [0, 0, 0, 0]
        print("Starting initialization sequence...")
        # [Fingers, Palm] abnormal code:
        # 10000 External encoder offset discrepancy
        # 01000 Failed to travel to home
        # 00100 Failed to fully close
        # 00010 Position out of range
        # 00001 home_offset abnormal

        # 0. Check PALM first
        if not self.bypass_DXL:
            # Compare DXL homing_offset
            if self.DC.get_homing_offset() != 0:
                print("Palm actuator needs calibration.\nCalibrate it first, or run with bypass_DXL option.")
                abnormal[3] = 0b00001
                return False
            else:
                # Check if position in range, home ~ home+pi/2
                palm_pos = self.DC.get_present_position()
                if (self.robot.palm.home - 0.2) < palm_pos < (self.robot.palm.home + 1.77):
                    pass
                else:
                    print("Palm actuator needs calibration.\nCalibrate it first, or run with bypass_DXL option.")
                    abnormal[3] = 0b00010
                    return False

        # bypass_DXL or PALM checked
        if self.bypass_DXL:
            input("Turn index fingers to parallel gesture then press enter.")
        else:
            print("Changing to Parallel gesture...")
            self.DC.torque_enable(1)
            self.DC.set_goal_position(self.robot.palm.home)
            time.sleep(0.5)
        self.robot.palm.angle = 0

        self.MC.init_driver_all()

        # 1. Compare home_offset
        for i in range(3):
            if round(self.robot.fingerlist[i].homing_offset, 2) != round(
                    self.MC.pbm.get_homing_offset(self.robot.finger_ids[i])[0][0][0], 2):
                abnormal[i] = abnormal[i] | 0b0001
                print("%s home_offset abnormal." % self.robot.fingerlist[i].name)
        # 2. Current position with in range
        present_pos = self.MC.pbm.get_present_position(BEAR_INDEX, BEAR_INDEX_M, BEAR_THUMB)
        for i, pos in enumerate(present_pos):
            if self.robot.fingerlist[i].mirrored:
                if pos[0][0] < -0.1 or pos[0][0] > self.robot.fingerlist[i].travel + 0.1:
                    abnormal[i] = abnormal[i] | 0b0010
                    print("%s present_pos out of range." % self.robot.fingerlist[i].name)
                    print(pos[0])
            else:
                if pos[0][0] > 0.1 or pos[0][0] < self.robot.fingerlist[i].travel - 0.1:
                    abnormal[i] = abnormal[i] | 0b0010
                    print("%s present_pos out of range." % self.robot.fingerlist[i].name)
                    print(pos[0])

        # # 3. Check for position Limit
        # limit_min = self.MC.pbm.get_limit_position_min(m_id)[0]
        # limit_max = self.MC.pbm.get_limit_position_max(m_id)[0]
        # if limit_min != end_pos or limit_max != 0:
        #     abnormal = True
        #     print("Position limit abnoraml")

        # Ask user if abnormal
        if sum(abnormal):
            usr = input("Fingers seem to need calibration. Do you want to continue anyway?(y/n)")
            if usr == "n" or usr == "N":
                return False
            else:
                pass

        # Set motor mode and PID
        # Set mode and limits
        self.MC.set_mode_all('position')

        # 4. Move to End -> complete
        running = [True, True, True]
        self.MC.torque_enable_all(1)
        self.MC.pbm.set_goal_position((BEAR_INDEX, self.robot.fingerlist[0].travel),
                                      (BEAR_INDEX_M, self.robot.fingerlist[1].travel),
                                      (BEAR_THUMB, self.robot.fingerlist[0].travel))
        start_time = time.time()
        while sum(running):
            try:
                status = self.MC.pbm.get_bulk_status((INDEX.motor_id, 'present_position', 'present_velocity'),
                                                     (INDEX_M.motor_id, 'present_position', 'present_velocity'),
                                                     (THUMB.motor_id, 'present_position', 'present_velocity'))
                err = [data[1] for data in status]
                position = [data[0][0] for data in status]
                # velocity = [data[0][1] for data in status]
                elapsed_time = time.time() - start_time
                if elapsed_time < TIMEOUT_INIT:
                    for i in range(3):
                        if running[i] and abs(position[i] - self.robot.fingerlist[i].travel) < 0.015:
                            running[i] = False
                            self.MC.damping_mode(self.robot.finger_ids[i])
                            print("%s end travel complete." % self.robot.fingerlist[i].name)
                        else:
                            self.MC.pbm.set_goal_position((self.robot.finger_ids[i], self.robot.fingerlist[i].travel))
                        if err[i] != 128 and err[i] != 144:
                            print("%s error, code:" % self.robot.fingerlist[i].name, bin(err[i]))
                else:
                    print("Timeout while moving to end. Is there something blocking the finger(s)?")
                    print("Abnormal:")
                    for i in range(3):
                        if running[i]:
                            abnormal[i] = abnormal[i] | 0b0100
                            print(self.robot.fingerlist[i].name)
                    self.MC.damping_mode_all()
                    running = [False, False, False]
            except KeyboardInterrupt:
                running = [0]
                print("User interrupted.")
        time.sleep(0.5)

        # 5. Move to Home -> complete
        print("Fingers resetting...")
        running = [True, True, True]
        # Enable torque and go to Home
        self.MC.damping_release_all()
        self.MC.torque_enable_all(1)
        self.MC.pbm.set_goal_position((THUMB.motor_id, 0),
                                      (INDEX.motor_id, 0),
                                      (INDEX_M.motor_id, 0))
        time.sleep(2)
        start_time = time.time()
        while sum(running):
            try:
                status = self.MC.pbm.get_bulk_status((INDEX.motor_id, 'present_position', 'present_velocity'),
                                                     (INDEX_M.motor_id, 'present_position', 'present_velocity'),
                                                     (THUMB.motor_id, 'present_position', 'present_velocity'))
                err = [data[1] for data in status]
                position = [data[0][0] for data in status]
                # velocity = [data[0][1] for data in status]
                elapsed_time = time.time() - start_time
                if elapsed_time < TIMEOUT_INIT:
                    for i in range(3):
                        if abs(position[i]) < 0.1:
                            running[i] = False
                            self.MC.torque_enable(self.robot.finger_ids[i], 0)
                        else:
                            self.MC.pbm.set_goal_position((self.robot.finger_ids[i], 0))
                        if err[i] != 128:
                            print("%s error, code:" % self.robot.fingerlist[i].name, bin(err[i]))
                else:
                    print("Timeout while resetting. Is there something blocking the finger(s)?")
                    print("Abnormal:")
                    for i in range(3):
                        if running[i]:
                            abnormal[i] = abnormal[i] | 0b1000
                            print(self.robot.fingerlist[i].name)
                            self.MC.torque_enable(self.robot.finger_ids[i], 0)
                    running = [False, False, False]
            except KeyboardInterrupt:
                running = [0]
                print("User interrupted.")

        # 6. Check external encoder reading and offset
        if not self.bypass_ext_enc:
            self.ext_enc.connect()
            time.sleep(0.2)
            ext_reading = self.ext_enc.get_angle()
            self.ext_enc.release()
            for idx, finger in enumerate(self.robot.fingerlist):
                if 0.18 < abs(ext_reading[idx] - finger.encoder_offset) < 6.1:
                    # External encoder reading differs from the record for too much
                    abnormal[idx] = abnormal[idx] | 0b10000
                    print("%s external encoder abnormal." % finger.name)
                    print("ext_reading: %f" % ext_reading[idx])
                    print("encoder_offset: %f" % finger.encoder_offset)

        # 7. Finger initialization complete
        # Disable and finish
        self.set_robot_enable(0)
        if sum(abnormal):
            print("Initialization failed.")
            for idx, code in enumerate(abnormal):
                if code:
                    print("%s abnormal, error code: %d" % (self.robot.fingerlist[idx].name, code))
            return False
        else:
            print("Initialization Complete.")
            for f in self.robot.fingerlist:
                f.initialized = True
            self.robot.palm.initialized = True
            self.robot.initialized = True
            # Get iq_compensation data
            iq_compensation = self.read_iq_compensation()
            if iq_compensation is None:
                # There is no iq compensation data
                print("WARNING: No iq_compensation found. Contact detection performance may be reduced.")
                usr = input("Do you want to run a quick calibration? (Y/n)")
                if usr == 'Y' or 'y':
                    # Run iq_compensation calibration
                    if self.calibration_iq_compensation():
                        self.robot.iq_compensation = self.read_iq_compensation()
                    else:
                        print("Iq compensation calibration failed.")
            else:
                self.robot.iq_compensation = iq_compensation
            return True

    def calibration_geometry(self):
        # Mark instance as un-started, uninitialized
        self.robot.booted = False
        self.robot.initialized = False

        # Calibrate the geometry of the whole hand.

        # Ping all actuators
        error = self.ping()  # 4 bit respectively for INDEX, INDEX_M, THUMB, Dynamixel

        if error:
            print("One or more actuators offline. Exiting...")
            return False

        if not self.bypass_DXL:
            # Calibrate Palm actuator first
            # Home INDEX finger
            usr = input(
                "Move all the finger to their home positions\nand put INDEX fingers in parallel, then press enter.")
            self.DC.set_homing_offset(0)
            self.robot.palm.home = self.DC.get_present_position()
            print("Palm home set.")
        else:
            self.robot.palm.home = 0

        self.MC.init_driver_all()

        # Set Mode
        self.MC.set_mode_all('velocity')

        # Set Limits
        for f in self.robot.fingerlist:
            # Limits
            self.MC.pbm.set_limit_iq_max((f.motor_id, 1.5))
            self.MC.pbm.set_limit_velocity_max((f.motor_id, 2 * VEL_CAL))

        # Home Dynamixel to the parallel position
        if self.bypass_DXL:
            usr = input("Move the index fingers to the parallel gesture then press enter.")
        else:
            # DXL not bypassed, automatic go to parallel.
            usr = input("Press ENTER to change to Pinch...")
            self.DC.set_mode("position")  # Position mode
            self.DC.torque_enable(1)  # Enable
            self.DC.set_goal_position(self.robot.palm.home + math.pi * 7 / 15)
            time.sleep(0.5)
            usr = input("If everything looks alright press enter to continue, INDEX fingers will return to Parallel.\n"
                        "Otherwise, press E and enter to exit.")
            if usr == 'E' or usr == 'e':
                return
            else:
                self.DC.set_goal_position(self.robot.palm.home)
                time.sleep(0.5)

        # Home finger
        # Get home
        usr = input("Auto set home? (y/n)")
        if usr == "n" or usr == "N":
            usr = input("Move all the finger to the home position then press enter.")

        elif usr == "y" or usr == "Y":
            # Look for home automatically
            running = [True, True, True]
            # Enable Torque
            self.MC.torque_enable_all(1)
            time.sleep(0.1)
            # Goal Velocity
            for f in self.robot.fingerlist:
                if f.mirrored:
                    goal_velocity = -VEL_CAL
                else:
                    goal_velocity = VEL_CAL
                self.MC.pbm.set_goal_velocity((f.motor_id, goal_velocity))

            home_i = [0, 0, 0]
            detect_count = [0, 0, 0]
            print("Looking for home...")
            while sum(running):
                try:
                    status = None
                    while status is None:
                        status = self.MC.pbm.bulk_read(self.robot.finger_ids, ['present_velocity', 'present_iq'],
                                                       error_mode=1)
                    # status = self.MC.pbm.get_bulk_status((INDEX.motor_id, 'present_velocity', 'present_iq'),
                    #                                      (INDEX_M.motor_id, 'present_velocity', 'present_iq'),
                    #                                      (THUMB.motor_id, 'present_velocity', 'present_iq'))
                    err = [i[1] for i in status]
                    vels = [i[0][0] for i in status]
                    iq = [i[0][1] for i in status]

                    for i in range(3):
                        if abs(vels[i]) < VEL_CAL_DETECT and abs(iq[i]) > IQ_CAL_DETECT:
                            if detect_count[i] > 5:
                                print("%s HOME reached." % self.robot.fingerlist[i].name)
                                # home_i[i] = iq[i]
                                # print("%s home_i acquired." % robot.fingerlist[i].name)
                                self.MC.torque_enable(self.robot.finger_ids[i], 0)
                                # print("End iq was %2.2f" % home_i[i])
                                # print("End velocity was %2.2f" % vels[i])
                                running[i] = False
                            else:
                                detect_count[i] += 1
                        if err[i] != 128:
                            print("%s BEAR error, code:" % self.robot.fingerlist[i].name, bin(err[i]))

                except KeyboardInterrupt:
                    running = [0]
                    self.MC.torque_enable_all(0)
                    print("User interrupted.")

        # Check if HOMING_OFFSET need to be updated
        finger_homing_error = []
        for idx, f in enumerate(self.robot.fingerlist):
            present_pos = self.MC.pbm.get_present_position(f.motor_id)[0][0][0]
            if abs(present_pos) > 0.15:
                # If the present position is away from zero for more than 0.05rad (~2.8deg)
                finger_homing_error.append([f, present_pos])
        if finger_homing_error:
            # If there is any finger that needs update
            print("The following finger's HOMING_OFFSET needs to be updated:")
            for i in finger_homing_error:
                print(i[0].name + ", homing error: " + str(i[1]))
            usr = input("Press ENTER to continue, or I if you insist to ignore.")
            if usr == 'I':
                finger_homing_error = []
        else:
            # No finger needs update
            print("All HOMING_OFFSET settings are fine.")
            usr = input("Press ENTER to continue, or C if you insist to reset HOMING_OFFSET for all fingers.")
            if usr == 'C':
                # User insist to reset HOMING_OFFSET for all fingers
                for f in self.robot.fingerlist:
                    finger_homing_error.append([f, 0])

        if finger_homing_error:
            # If there is any finger to be updated
            # Clear HOMING_OFFSET
            for i in finger_homing_error:
                m_id = i[0].motor_id
                self.MC.pbm.set_homing_offset((m_id, 0))
                # Check setting
                check = False
                trial_count = 1
                # debug_temp = motor_controller.pbm.get_homing_offset(m_id, m_id, m_id)
                while not check:
                    try:
                        if abs(self.MC.pbm.get_homing_offset(m_id)[0][0][0]) < 1:
                            check = True
                            print("HOMING_OFFSET cleared for %s. Trails: %d." % (i[0].name, trial_count))
                        else:
                            self.MC.pbm.set_homing_offset((m_id, 0))
                            time.sleep(0.05)
                            trial_count += 1
                    except KeyboardInterrupt:
                        check = True
                        print("User interrupted.")
            # Wait for 0.2 sec after setting HOMING_OFFSET
            time.sleep(0.2)

            # Get home_offset
            for i in finger_homing_error:
                i[0].homing_offset = -(self.MC.pbm.get_present_position(i[0].motor_id)[0][0][0])
            # print(home_offset)

            # Set Homing_Offset
            for i in finger_homing_error:
                m_id = i[0].motor_id
                homing_offset = i[0].homing_offset
                self.MC.pbm.set_homing_offset((m_id, homing_offset))
                # time.sleep(0.05)
                # Check setting
                check = False
                trial_count = 1
                while not check:
                    try:
                        temp = self.MC.pbm.get_homing_offset(m_id)[0][0][0]
                        print("Current homing_offset: % 2.2f" % temp)
                        if abs(self.MC.pbm.get_homing_offset(m_id)[0][0][0] - homing_offset) < 0.01:
                            check = True
                            print("HOMING_OFFSET updated for %s. Trails: %d." % (i[0].name, trial_count))
                        else:
                            self.MC.pbm.set_homing_offset((m_id, homing_offset))
                            # time.sleep(0.05)
                            trial_count += 1
                    except KeyboardInterrupt:
                        check = True
                        print("User interrupted.")
            # Wait for 0.2 sec after setting HOMING_OFFSET
            time.sleep(0.2)
            print("HOMING_OFFSET all updated.")

            # Final check
            for i in finger_homing_error:
                m_id = i[0].motor_id
                pos = self.MC.pbm.get_present_position(m_id)[0][0][0]
                if abs(pos) < 0.01:
                    print("%s homed." % i[0].name)
                else:
                    print("%s homing abnormal!" % i[0].name)  # TODO: throw exception
                    usr = input("Continue? (Y/N)")
                    if usr == 'Y' or 'y':
                        pass
                    else:
                        return
        # Update all finger.homing_offset
        for f in self.robot.fingerlist:
            f.homing_offset = self.MC.pbm.get_homing_offset(f.motor_id)[0][0][0]

        if not self.bypass_ext_enc:
            self.ext_enc.connect()
            time.sleep(0.5)
            data = self.ext_enc.get_angle()
            self.ext_enc.release()
            for idx, f in enumerate(self.robot.fingerlist):
                f.encoder_offset = data[idx]

        # Get end_pos
        usr = input("Auto set the end limit? (y/n)")
        if usr == "n" or usr == "N":
            usr = input("Move all the fingers to the end limit and press enter.\n")
            end_pos = self.MC.pbm.get_present_position(INDEX.motor_id, INDEX_M.motor_id, THUMB.motor_id)
            for idx, i in enumerate(end_pos):
                self.robot.fingerlist[idx].travel = i[0][0]
                print("end_pos recorded as:", self.robot.fingerlist[idx].name, self.robot.fingerlist[idx].travel)

        elif usr == "y" or usr == "Y":
            # Look for End-Pos automatically
            running = [True, True, True]
            # Enable Torque
            self.MC.torque_enable_all(1)
            time.sleep(0.1)
            # Goal Velocity
            for f in self.robot.fingerlist:
                if f.mirrored:
                    goal_velocity = VEL_CAL
                else:
                    goal_velocity = -VEL_CAL
                self.MC.pbm.set_goal_velocity((f.motor_id, goal_velocity))

            time.sleep(0.5)
            end_i = [0, 0, 0]
            detect_count = [0, 0, 0]
            print("Looking for end_pos...")
            while sum(running):
                try:
                    status = None
                    while status is None:
                        status = self.MC.pbm.bulk_read(self.robot.finger_ids, ['present_velocity', 'present_iq'],
                                                       error_mode=1)
                    # status = motor_controller.pbm.get_bulk_status((INDEX.motor_id, 'present_velocity', 'present_iq'),
                    #                                               (INDEX_M.motor_id, 'present_velocity', 'present_iq'),
                    #                                               (THUMB.motor_id, 'present_velocity', 'present_iq'))
                    err = [i[1] for i in status]
                    vels = [i[0][0] for i in status]
                    iq = [i[0][1] for i in status]

                    for i in range(3):
                        if abs(vels[i]) < VEL_CAL_DETECT and abs(iq[i]) > IQ_CAL_DETECT:
                            if detect_count[i] > 20:
                                # TODO: check total travel value
                                print("%s end_pos reached." % self.robot.fingerlist[i].name)
                                self.robot.fingerlist[i].travel = \
                                    self.MC.pbm.get_present_position(self.robot.finger_ids[i])[0][0][0]
                                print("%s end_pos acquired." % self.robot.fingerlist[i].name)
                                end_i[i] = iq[i]
                                print("%s end_i acquired." % self.robot.fingerlist[i].name)
                                self.MC.torque_enable(self.robot.finger_ids[i], 0)
                                print("%s end_pos recorded as: % 8.2f" %
                                      (self.robot.fingerlist[i].name, self.robot.fingerlist[i].travel))
                                print("End iq was %2.2f" % end_i[i])
                                print("End velocity was %2.2f" % vels[i])
                                running[i] = False
                            else:
                                detect_count[i] += 1
                        if err[i] != 128:
                            print("%s BEAR error, code:" % self.robot.fingerlist[i].name, bin(err[i]))

                except KeyboardInterrupt:
                    running = [0]
                    self.MC.torque_enable_all(0)
                    end_pos = [0, 0, 0]
                    print("User interrupted.")

        usr = input("Reset fingers? (y/n)")
        if usr == "y" or usr == "Y":
            for f in self.robot.fingerlist:
                self.MC.pbm.set_p_gain_position((f.motor_id, POS_P))
                self.MC.pbm.set_i_gain_position((f.motor_id, POS_I))
                self.MC.pbm.set_d_gain_position((f.motor_id, POS_D))
                self.MC.pbm.set_limit_iq_max((f.motor_id, 1))
            time.sleep(0.2)
            print("Fingers resetting...")

            # Set Mode and Limit
            self.MC.set_mode_all('position')

            running = [True, True, True]
            # Enable torque and go to Home
            self.MC.torque_enable_all(1)
            self.MC.pbm.set_goal_position((THUMB.motor_id, 0.01), (INDEX.motor_id, 0.01), (INDEX_M.motor_id, 0.01))
            time.sleep(1)
            while sum(running):
                try:
                    # status = self.MC.pbm.get_bulk_status(
                    #     (INDEX.motor_id, 'present_position'),
                    #     (INDEX_M.motor_id, 'present_position'),
                    #     (THUMB.motor_id, 'present_position'))
                    status = self.MC.pbm.get_present_position(INDEX.motor_id, INDEX_M.motor_id, THUMB.motor_id)
                    err = [data[1] for data in status]
                    position = [data[0][0] for data in status]
                    for i in range(3):
                        if abs(position[i]) < 0.08:
                            running[i] = False
                            self.MC.torque_enable(self.robot.finger_ids[i], 0)
                        if err[i] != 128:
                            print("%s BEAR error, code:" % self.robot.fingerlist[i].name, bin(err[i]))
                except KeyboardInterrupt:
                    running = [0]
                    print("User interrupted.")

        self.MC.torque_enable_all(0)
        if not self.bypass_DXL:
            self.DC.torque_enable(0)

        print("Calibration summary:")
        for f in self.robot.fingerlist:
            print("%s Motor ID: %d, homing_offset: % 8.2f, travel: % 8.2f, encoder_offset: % 8.2f"
                  % (f.name, f.motor_id, f.homing_offset, f.travel, f.encoder_offset))
        if finger_homing_error:
            print("The following fingers' homing_offset has been updated:")
            print([i[0].name for i in finger_homing_error])
            usr = input("It is recommended to save changed settings. Continue? (y/n)")
            if usr == 'Y' or 'y':
                for i in finger_homing_error:
                    self.MC.pbm.save_config(i[0].motor_id)
                print("Config saved to BEAR.")
            else:
                print("Abort saving to BEAR. Please power cycle hardware before using.")

        usr = input("Save Data? (y/n)")
        if usr == "y" or usr == "Y":
            # Build list
            data = []
            for f in self.robot.fingerlist:
                data.append([f.name, f.motor_id, f.homing_offset, f.travel, f.encoder_offset])
            data.append([self.robot.palm.name, self.robot.palm.motor_id, self.robot.palm.home, self.robot.palm.travel])
            # Write file
            filename = 'Settings/initials.txt'
            filepath = os.path.join(str(Path(os.getcwd())), filename)
            initials = open(filepath, 'w')
            for i in data:
                initials.write(str(i)[1:-1])
                initials.write('\n')
            initials.close()
            print("Full hand calibration complete.")
        else:
            print("Abort saving to initials.txt.")

    def calibration_iq_compensation(self, plot=False):
        """Calibrate for rough iq_compensation for more accurate contact possibility"""
        # Robot must be booted and initialized
        # Check initialization
        if not self.robot.initialized:
            print("Robot not initialized. Please run initialization() first.")
            return False
        else:
            self.change_gesture("P")
            # Quasi-static position tracking to figure out iq comp for spring
            self.MC.set_mode_all('force')
            self.set_stiffness(3, 45, 1)

            # Generate trajectory
            finger_traj = []
            sample_rate = 1000
            for finger in self.robot.fingerlist:
                finger_traj.append(np.linspace(0, finger.travel, sample_rate))

            self.set_robot_enable(1)
            # Send all fingers to 0 position
            self.MC.pbm.bulk_write([BEAR_INDEX, BEAR_INDEX_M, BEAR_THUMB], ['goal_position'],
                                   [[finger_traj[0][99]], [finger_traj[1][99]], [finger_traj[2][99]]])
            time.sleep(1)

            # Now move along trajectory and collect data
            data = []
            t = []
            finger_pos = [[], [], []]
            finger_velocity = [[], [], []]
            finger_iq = [[], [], []]
            start_time = time.time()
            for i in range(100, 800):
                data.clear()
                data = self.MC.pbm.bulk_read_write([BEAR_INDEX, BEAR_INDEX_M, BEAR_THUMB],
                                                   ['present_position', 'present_velocity', 'present_iq'],
                                                   ['goal_position'],
                                                   [[finger_traj[0][i]], [finger_traj[1][i]], [finger_traj[2][i]]])
                t.append(time.time() - start_time)
                for j in range(3):
                    finger_pos[j].append(data[j][0][0])
                    finger_velocity[j].append(data[j][0][1])
                    finger_iq[j].append(data[j][0][2])
                time.sleep(0.001)

            # Get last data
            data.clear()
            data = self.MC.pbm.bulk_read([BEAR_INDEX, BEAR_INDEX_M, BEAR_THUMB],
                                         ['present_position', 'present_velocity', 'present_iq'])
            t.append(time.time() - start_time)
            for j in range(3):
                finger_pos[j].append(data[j][0][0])
                finger_velocity[j].append(data[j][0][1])
                finger_iq[j].append(data[j][0][2])

            time.sleep(0.5)
            self.set_stiffness(3, 5, 1)
            self.release('F')
            self.set_robot_enable(0)

            finger_iq_filtered = [[0] * 701, [0] * 701, [0] * 701]
            for i in range(len(finger_velocity[0])):
                for j in range(3):
                    if i == 0:
                        finger_iq_filtered[j][i] = finger_iq[j][i]
                    else:
                        finger_iq_filtered[j][i] = SMOOTHING_IQ * finger_iq[j][i] + \
                                                   (1 - SMOOTHING_IQ) * finger_iq_filtered[j][i - 1]
            avrg_iq = []
            for data in finger_iq:
                avrg_iq.append(sum(data) / len(data))

            if plot:
                plt.figure(1)
                plt.subplot(311)
                plt.plot(t, finger_iq_filtered[0], 'g', t, finger_iq[0], 'c', t, avrg_iq[0] * 701, 'k')
                plt.grid(True)
                plt.subplot(312)
                plt.plot(t, finger_iq_filtered[1], 'g', t, finger_iq[1], 'c', t, avrg_iq[1] * 701, 'k')
                plt.grid(True)
                plt.subplot(313)
                plt.plot(t, finger_iq_filtered[2], 'g', t, finger_iq[2], 'c', t, avrg_iq[2] * 701, 'k')
                plt.grid(True)
                plt.show()

            # Write to file
            print("Calibration result:", avrg_iq)
            usr = input("Save Data? (y/n)")
            if usr == "y" or usr == "Y":
                filename = 'Settings/iq_compensation.txt'
                filepath = os.path.join(str(Path(os.getcwd())), filename)
                iq_comp = open(filepath, 'w')
                for i in avrg_iq:
                    iq_comp.write(str(i))
                    iq_comp.write(',')
                iq_comp.close()
                self.robot.iq_compensation = avrg_iq
                print("Iq compensation calibration complete.")
                return True
            else:
                print("Abort saving to initials.txt.")
                return False

    # ------------------------
    # MOTION FUNCTIONS
    # ------------------------
    # Functions for DAnTE motions
    # Change gesture
    # Control grab and release motion of self.robot

    def change_gesture(self, new_gesture):
        """
        Change the gesture of DT02

        :param new_gesture: Y-Tripod, I-Pinch, P-Parallel,
        :type new_gesture: str
        """
        # TODO: Create shallow_release function so that the index fingers release to slightly away from home for
        #  gesture change, then fully release.
        # Change the gesture of DAnTE
        # Will Fully release first.

        # Check initialization
        if not self.robot.initialized:
            error = 3
            print("Robot not initialized. Exit.")
            return error

        # Check enable first
        if self.get_robot_enable():
            pass
        else:
            print("WARNING: Robot not enabled, enabling now.")
            self.set_robot_enable(1)

        # if self.gesture == new_gesture:
        if self.robot.palm.gesture == new_gesture:
            # No need to change
            print('Already in gesture %s.' % new_gesture)
            return True
        if new_gesture not in ['Y', 'I', 'P']:
            # Check for invalid input
            print("Invalid input.")  # TODO: Throw an exception
            return False

        else:
            # change the new_gesture of self.robot to: tripod(Y), pinch(I) or parallel(P)
            # Reset all fingers under present gesture before changing gesture
            self.release('F')

            # Gesture change operations
            if new_gesture == 'Y':
                # Change to tripod
                print("Changing to Tripod.")
                if self.bypass_DXL:
                    input("Bypass_DXL, please manually change DAnTE to Tripod mode, and press enter.")
                else:
                    DXL_goal_pos = self.robot.palm.home + math.pi / 3
                    self.robot.palm.angle = math.pi / 3
                    self.DC.set_goal_position(DXL_goal_pos)
                # self.gesture = 'Y'
                # Update balance_factor
                self.balance_factor = [1, 1, 1]

            elif new_gesture == 'I':
                # Change to pinch
                print("Changing to Pinch.")
                if self.bypass_DXL:
                    input("Bypass_DXL, please manually change DAnTE to Pinch mode, and press enter.")
                else:
                    DXL_goal_pos = self.robot.palm.home + math.pi / 2 * (15 / 16)
                    self.robot.palm.angle = math.pi / 2
                    self.DC.set_goal_position(DXL_goal_pos)
                    time.sleep(0.5)
                # self.gesture = 'I'
                # Update balance_factor
                self.balance_factor = [1, 1, 1]
                # Set THUMB in IDLE
                self.idle(THUMB)
            else:
                # Change to parallel
                print("Changing to Parallel.")
                if self.bypass_DXL:
                    input("Bypass_DXL, please manually change DAnTE to Parallel mode, and press enter.")
                else:
                    DXL_goal_pos = self.robot.palm.home
                    self.robot.palm.angle = 0
                    self.DC.set_goal_position(DXL_goal_pos)
                # self.gesture = 'P'
                # Update balance_factor
                self.balance_factor = [1, 1, 2]

            # Check if DXL still moving before exiting
            if not self.bypass_DXL:
                running = True
                while running:
                    try:
                        if self.DC.get_present_velocity() < 0.05:
                            time.sleep(0.25)
                            running = False
                    except KeyboardInterrupt:
                        print("User interrupted.")
                        running = False
                        return False
            # Update robot.palm
            self.robot.palm.gesture = new_gesture
            return True

    def grab(self, gesture, mode, **options):
        # Trying a new method of contact detection based on possibility calcultated from velocity and iq
        """
        Control grab motion of self.robot
        If keyword options not specified, it will go with a default. See Macros_DAnTE

        :param str gesture: Grab with a gesture, tripod(Y), pinch(I) or parallel(P)
        :param str mode: Specify a grab mode: (H)old or (G)rip

        :keyword float approach_speed: Approach speed
        :keyword float approach_stiffness: Approach stiffness
        :keyword float final_stiffness: Grip/Hold stiffness
        :keyword float preload: Grip/Hold reload, preload in Amps.
        :keyword float max_iq: Maximum torque current
        :keyword bool logging: Data log function on/off, debug purpose.
        """

        error = 0  # 1 for timeout, 2 for user interruption, 3 for initialization, 9 for Invalid input

        # Check initialization
        if not self.robot.initialized:
            error = 3
            print("Robot not initialized. Exit.")
            return error

        # Check enable and enable system if not.
        if not self.get_robot_enable():
            self.MC.torque_enable_all(1)

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
            self.mode = mode
            # Options:
            self.approach_speed = max(options.get("approach_speed", default_approach_speed), approach_speed_min)
            self.approach_stiffness = max(options.get("approach_stiffness", default_approach_stiffness),
                                          approach_stiffness_min)
            self.final_stiffness = options.get("final_stiffness", self.approach_stiffness)
            self.preload = options.get("preload", default_preload)
            self.max_iq = max(options.get("max_iq", default_max_iq), 2*self.preload)
            logging = options.get("logging", False)

            # update approach stiffness
            self.approach_stiffness = approach_stiffness_func(self.approach_speed, self.approach_stiffness)

        # Calculate approach P and I gains from approach_stiffness
        approach_p = approach_p_func(self.approach_stiffness)
        approach_i = approach_i_func(self.approach_stiffness)

        contact_count = 0

        # Calculate goal_approach_speed
        goal_approach_speed = [-self.approach_speed + 2 * self.approach_speed * f.mirrored for f in
                               self.robot.fingerlist]

        # Start with change into gesture
        self.change_gesture(gesture)
        # Set fingers' Direct Force PID according to Stiffness
        # self.set_approach_stiffness()
        force_p = self.approach_stiffness
        force_d = min(max(0.1 * force_p, force_d_min), force_d_max)
        finger_count = self.get_finger_count()
        self.set_stiffness(finger_count, force_p, force_d)

        # Set into Direct Force Mode
        self.MC.set_mode_all('force')

        # Set iq_max
        for f_id in self.robot.finger_ids:
            self.MC.pbm.set_limit_iq_max((f_id, self.max_iq))
        if self.robot.palm.gesture == 'P':
            # Double THUMB iq_limit in Parallel mode
            self.MC.pbm.set_limit_iq_max((THUMB.motor_id, 2 * self.max_iq))
            # Enforce writing
            check = False
            while not check:
                if round(self.MC.pbm.get_limit_iq_max(THUMB.motor_id)[0][0][0], 4) == round(2 * self.max_iq, 4):
                    check = True

        # usr = input("Press enter to grab...")

        # 3. Fingers close tracking approach_speed, switch to final grip/hold upon object detection
        if self.robot.contact:
            print("Please release contact first.")
            return
        else:
            if self.robot.palm.gesture == 'I':
                finger_count = 2
            else:
                finger_count = 3
            print("Approaching...")
            # Get start_time
            # Python Version 3.7 or above
            # start_time = time.time_ns()/1000000000  # In ns unit
            start_time = time.time()
            present_time = start_time

            # Initialize status variables
            velocity = [0, 0, 0]
            prev_velocity = [0, 0, 0]
            velocity_error = [0, 0, 0]
            velocity_error_int = [0, 0, 0]
            iq = [0, 0, 0]
            goal_iq = [0, 0, 0]
            position = [0, 0, 0]
            goal_position = self.MC.get_present_position_all()  # populate with present pos
            possibility = [0, 0, 0]
            possibility_int = [0, 0, 0]  # integration of relatively low possibility
            criteria = self.contact_possibility_criteria()  # update criteria

            # RAW(unfiltered data)
            vel_raw = [0, 0, 0]
            iq_raw = [0, 0, 0]

            # iq window
            iq_window = [deque([0] * 20), deque([0] * 20), deque([0] * 20)]  # windowed queue for present iq of size 20

            # Status logging
            velocity_log = []
            position_log = []
            goal_position_log = []
            time_log = []
            delta_time_log = []
            iq_log = []
            iq_comp_log = []
            possibility_log = []
            vel_raw_log = []
            iq_raw_log = []
            contact_log = []

            delta_time = 0
            time.sleep(0.001)  # Delay 1ms so that we don't get a zero delta_time for the first loop.

            delta_time_sum = 0
            loop_count = 0

            while not (error or self.robot.contact):
                # Main GRAB loop
                try:
                    # Get time stamp
                    previous_time = present_time
                    present_time = time.time()
                    delta_time = present_time - previous_time
                    grab_time = present_time - start_time

                    # Collect status and send command
                    status = self.MC.grab_loop_comm(self.robot.palm.gesture, goal_position, goal_iq)

                    # print(time.time()-present_time)

                    # Process data
                    # print(status)
                    # Motor Error
                    motor_err = [i[1] for i in status]
                    # Position
                    position = [i[0][0] for i in status]
                    # iq
                    iq = [SMOOTHING_IQ * status[i][0][2] + (1 - SMOOTHING_IQ) * iq[i] for i in range(finger_count)]
                    iq_raw = [status[i][0][2] for i in range(finger_count)]
                    for idx in range(finger_count):
                        iq_window[idx].append(iq[idx])
                        iq_window[idx].popleft()
                    # Velocity
                    prev_velocity = velocity
                    velocity = [SMOOTHING_VEL * status[i][0][1] + (1 - SMOOTHING_VEL) * velocity[i] for i in
                                range(finger_count)]
                    vel_raw = [status[i][0][1] for i in range(finger_count)]

                    # Approach motion
                    # Calculate approach_command
                    # RobotController PID generating position command so that finger tracks approach_speed
                    # Build approach commands
                    velocity_error = [goal_approach_speed[i] - velocity[i] for i in range(finger_count)]
                    velocity_error_int = [velocity_error[i] * delta_time + velocity_error_int[i]
                                          for i in range(finger_count)]

                    # Determine if contact and switch to torque mode and temporarily maintain iq_compensation
                    # Calculate goal_iq/goal_position and switch mode for contacted
                    # Just send both goal commands, let BEAR choose which to use.
                    for idx in range(finger_count):
                        if not self.robot.fingerlist[idx].contact:
                            # Keep it running for now
                            # Get possibility
                            possibility[idx] = self.contact_possibility(abs(iq[idx] - self.robot.iq_compensation[idx]),
                                                                        abs(velocity[idx]),
                                                                        grab_time)
                            if possibility[idx] > criteria[0]:
                                possibility_int[idx] += possibility[idx]
                            if possibility[idx] > criteria[1] or possibility_int[idx] > criteria[2]:
                                # This finger has come to contact
                                self.robot.fingerlist[idx].contact = True
                                print('Finger contact:', self.robot.fingerlist[idx].name, 'Position:', position[idx])
                                self.contact_position[idx] = position[idx]
                                goal_position[idx] = self.contact_position[idx]
                                goal_iq[idx] = self.robot.iq_compensation[idx]
                                self.MC.set_mode(self.robot.fingerlist[idx].motor_id, 'torque')
                                contact_count += 1
                            else:
                                # Build command
                                goal_position[idx] = position[idx] + approach_p * velocity_error[idx] \
                                                     + approach_i * velocity_error_int[idx]
                    # bulk_read_write at loop head.
                    self.robot.contact = contact_count == finger_count

                    # Data logging
                    if logging:
                        delta_time_log.append(delta_time)
                        time_log.append(grab_time)
                        velocity_log.append(velocity[:])
                        position_log.append(position[:])
                        goal_position_log.append(goal_position[:])
                        iq_log.append(iq[:])
                        possibility_log.append(possibility[:])
                        vel_raw_log.append(vel_raw[:])
                        iq_raw_log.append(iq_raw[:])
                        contact_log.append([f.contact for f in self.robot.fingerlist])

                    # Check for timeout
                    if grab_time > TIMEOUT_GRAB:
                        print("Grab motion timeout")
                        self.MC.torque_enable_all(0)
                        error = 1

                    loop_count += 1
                    delta_time_sum += delta_time

                except KeyboardInterrupt:
                    print("User interrupted.")
                    running = False
                    error = 2

            # Out of while loop -> error or contact
            # Write goal status one more time so that the last contact finger gets its goal iq command
            self.MC.set_goal_iq(self.robot.palm.gesture, goal_iq)

            avg_loop_time = delta_time_sum / loop_count
            print("Average loop time: %f" % avg_loop_time)

            # Data processing -logging
            if logging:
                # Format all data so that it is in this formation:
                # data_log_all = [[INDEX data], [INDEX_M data], [THUMB data]]
                velocity_log_all = []
                position_log_all = []
                goal_position_log_all = []
                iq_log_all = []
                iq_raw_log_all = []
                vel_raw_log_all = []
                possibility_log_all = []
                contact_log_all = []

                for i in range(3):
                    velocity_log_all.append([data[i] for data in velocity_log])
                    vel_raw_log_all.append([data[i] for data in vel_raw_log])
                    position_log_all.append([data[i] for data in position_log])
                    goal_position_log_all.append([data[i] for data in goal_position_log])
                    iq_log_all.append([data[i] for data in iq_log])
                    iq_raw_log_all.append([data[i] for data in iq_raw_log])
                    possibility_log_all.append([data[i] for data in possibility_log])
                    contact_log_all.append([data[i] for data in contact_log])

                # # Plot here
                plt.figure(1)
                plt.subplot(311)
                plt.plot(time_log, position_log_all[0], 'b',
                         time_log, goal_position_log_all[0], 'c',
                         time_log, velocity_log_all[0], 'r',
                         time_log, contact_log_all[0], 'k')
                plt.grid(True)

                plt.subplot(312)
                plt.plot(time_log, position_log_all[1], 'b',
                         time_log, goal_position_log_all[1], 'c',
                         time_log, velocity_log_all[1], 'r',
                         time_log, contact_log_all[1], 'k')
                plt.grid(True)

                plt.subplot(313)
                plt.plot(time_log, position_log_all[2], 'b',
                         time_log, goal_position_log_all[1], 'c',
                         time_log, velocity_log_all[2], 'r',
                         time_log, contact_log_all[2], 'k')
                plt.grid(True)
                plt.show()

            if error:
                self.MC.torque_enable_all(0)
                print("Grab error. System disabled.")
                return error
            else:
                # # Get contact iq for all fingers
                # self.contact_iq = [sum(data) / 20 for data in iq_window]
                # Run grab_end motion
                self.grab_end(logging=logging)

    def grab_simplified(self, gesture, mode, **options):
        # Trying a new method of contact detection based on possibility calcultated from velocity and iq
        # Trying a simplified method of just using velocity mode to approach
        """
        Control grab motion of self.robot
        If keyword options not specified, it will go with a default. See Macros_DAnTE

        :param str gesture: Grab with a gesture, tripod(Y), pinch(I) or parallel(P)
        :param str mode: Specify a grab mode: (H)old or (G)rip

        :keyword float approach_speed: Approach speed
        :keyword float approach_stiffness: Approach stiffness
        :keyword float final_stiffness: Grip/Hold stiffness
        :keyword float preload: Grip/Hold reload, preload in Amps.
        :keyword float max_iq: Maximum torque current
        :keyword bool logging: Data log function on/off, debug purpose.
        """

        error = 0  # 1 for timeout, 2 for user interruption, 3 for initialization, 9 for Invalid input

        # Check initialization
        if not self.robot.initialized:
            error = 3
            print("Robot not initialized. Exit.")
            return error

        # Check enable and enable system if not.
        if not self.get_robot_enable():
            self.MC.torque_enable_all(1)

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
            self.mode = mode
            # Options:
            self.approach_speed = max(options.get("approach_speed", default_approach_speed), approach_speed_min)
            self.approach_stiffness = max(options.get("approach_stiffness", default_approach_stiffness),
                                          approach_stiffness_min)
            self.final_stiffness = options.get("final_stiffness", self.approach_stiffness)
            self.preload = options.get("preload", default_preload)
            self.max_iq = max(options.get("max_iq", default_max_iq), 2*self.preload)
            logging = options.get("logging", False)

            # update approach stiffness
            self.approach_stiffness = approach_stiffness_func(self.approach_speed, self.approach_stiffness)

        # Calculate approach P and I gains from approach_stiffness
        approach_p = approach_p_func(self.approach_stiffness)
        approach_i = approach_i_func(self.approach_stiffness)

        contact_count = 0

        # Calculate goal_approach_speed
        goal_approach_speed = [-self.approach_speed + 2 * self.approach_speed * f.mirrored for f in
                               self.robot.fingerlist]
        # Calculate preload with sign
        goal_preload = [-self.preload + 2 * self.preload * f.mirrored for f in self.robot.fingerlist]

        # Start with change into gesture
        self.change_gesture(gesture)
        # Set fingers' Direct Force PID according to Stiffness
        # self.set_approach_stiffness()
        force_p = self.approach_stiffness
        force_d = min(max(0.1 * force_p, force_d_min), force_d_max)
        finger_count = self.get_finger_count()
        self.set_stiffness(finger_count, force_p, force_d)

        # Set into Velocity Mode
        self.MC.set_mode_all('velocity')
        # TODO: get back to DF Mode
        # # Set into Direct Force Mode
        # self.MC.set_mode_all('force')

        # Set i_max and velocity_max
        for f_id in self.robot.finger_ids:
            self.MC.pbm.set_limit_i_max((f_id, approach_i_limit))
            self.MC.pbm.set_limit_velocity_max((f_id, self.approach_speed+0.2))

        # usr = input("Press enter to grab...")

        # 3. Fingers close tracking approach_speed, switch to final grip/hold upon object detection
        if self.robot.contact:
            print("Please release contact first.")
            return
        else:
            if self.robot.palm.gesture == 'I':
                finger_count = 2
                goal_approach_speed[2] = 0
                goal_preload[2] = 0
            else:
                finger_count = 3
            print("Approaching...")

            # Get start_time
            # Python Version 3.7 or above
            # start_time = time.time_ns()/1000000000  # In ns unit
            start_time = time.time()
            present_time = start_time

            # Initialize status variables
            velocity = [0, 0, 0]
            prev_velocity = [0, 0, 0]
            velocity_error = [0, 0, 0]
            velocity_error_int = [0, 0, 0]
            iq = [0, 0, 0]
            goal_iq = [0, 0, 0]
            position = [0, 0, 0]
            goal_position = self.MC.get_present_position_all()  # populate with present pos
            possibility = [0, 0, 0]
            possibility_int = [0, 0, 0]  # integration of relatively low possibility
            criteria = self.contact_possibility_criteria()  # update criteria

            # RAW(unfiltered data)
            vel_raw = [0, 0, 0]
            iq_raw = [0, 0, 0]

            # iq window
            iq_window = [deque([0] * 20), deque([0] * 20), deque([0] * 20)]  # windowed queue for present iq of size 20

            # Status logging
            velocity_log = []
            position_log = []
            goal_position_log = []
            time_log = []
            delta_time_log = []
            iq_log = []
            iq_comp_log = []
            possibility_log = []
            vel_raw_log = []
            iq_raw_log = []
            contact_log = []

            delta_time = 0
            time.sleep(0.001)  # Delay 1ms so that we don't get a zero delta_time for the first loop.

            delta_time_sum = 0
            loop_count = 0

            while not (error or self.robot.contact):
                # Main GRAB loop
                try:
                    # Get time stamp
                    previous_time = present_time
                    present_time = time.time()
                    delta_time = present_time - previous_time
                    grab_time = present_time - start_time

                    # Collect status and send velocity command
                    status = self.MC.grab_loop_comm_all(self.robot.palm.gesture,
                                                        goal_position, goal_approach_speed, goal_iq)

                    # print(time.time()-present_time)

                    # Process data
                    # print(status)
                    # Motor Error
                    motor_err = [i[1] for i in status]
                    # Position
                    position = [i[0][0] for i in status]
                    # iq
                    iq = [SMOOTHING_IQ * status[i][0][2] + (1 - SMOOTHING_IQ) * iq[i] for i in range(finger_count)]
                    iq_raw = [status[i][0][2] for i in range(finger_count)]
                    for idx in range(finger_count):
                        iq_window[idx].append(iq[idx])
                        iq_window[idx].popleft()
                    # Velocity
                    prev_velocity = velocity
                    velocity = [SMOOTHING_VEL * status[i][0][1] + (1 - SMOOTHING_VEL) * velocity[i] for i in
                                range(finger_count)]
                    vel_raw = [status[i][0][1] for i in range(finger_count)]

                    # Approach motion
                    # Calculate approach_command
                    # RobotController PID generating position command so that finger tracks approach_speed
                    # Build approach commands
                    velocity_error = [goal_approach_speed[i] - velocity[i] for i in range(finger_count)]
                    velocity_error_int = [velocity_error[i] * delta_time + velocity_error_int[i]
                                          for i in range(finger_count)]

                    # Determine if contact and switch to force mode and maintain contact position, 0 velocity and
                    # preload
                    # Just send all goal commands, let BEAR choose which to use.
                    for idx in range(finger_count):
                        if not self.robot.fingerlist[idx].contact:
                            # Keep it running for now
                            # Get possibility
                            possibility[idx] = self.contact_possibility(abs(iq[idx] - self.robot.iq_compensation[idx]),
                                                                        abs(velocity[idx]),
                                                                        grab_time)
                            if possibility[idx] > criteria[0]:
                                possibility_int[idx] += possibility[idx]
                            if possibility[idx] > criteria[1] or possibility_int[idx] > criteria[2]:
                                # This finger has come to contact
                                # Switch to DF mode and maintain contact position, 0 velocity and preload
                                self.robot.fingerlist[idx].contact = True
                                print('Finger contact:', self.robot.fingerlist[idx].name, 'Position:', position[idx])
                                self.contact_position[idx] = position[idx]
                                goal_position[idx] = self.contact_position[idx]
                                goal_iq[idx] = self.robot.iq_compensation[idx]+goal_preload[idx]
                                self.MC.set_mode(self.robot.fingerlist[idx].motor_id, 'force')
                                contact_count += 1
                    # bulk_read_write at loop head.
                    self.robot.contact = contact_count == finger_count

                    # Data logging
                    if logging:
                        delta_time_log.append(delta_time)
                        time_log.append(grab_time)
                        velocity_log.append(velocity[:])
                        position_log.append(position[:])
                        goal_position_log.append(goal_position[:])
                        iq_log.append(iq[:])
                        possibility_log.append(possibility[:])
                        vel_raw_log.append(vel_raw[:])
                        iq_raw_log.append(iq_raw[:])
                        contact_log.append([f.contact for f in self.robot.fingerlist])

                    # Check for timeout
                    if grab_time > TIMEOUT_GRAB:
                        print("Grab motion timeout")
                        self.MC.torque_enable_all(0)
                        error = 1

                    loop_count += 1
                    delta_time_sum += delta_time

                except KeyboardInterrupt:
                    print("User interrupted.")
                    running = False
                    error = 2

            # Out of while loop -> error or contact
            # Write goal status one more time so that the last contact finger gets its goal iq command
            self.MC.grab_loop_write_all(self.robot.palm.gesture, goal_position, goal_approach_speed, goal_iq)

            avg_loop_time = delta_time_sum / loop_count
            print("Average loop time: %f" % avg_loop_time)

            # Data processing -logging
            if logging:
                # Format all data so that it is in this formation:
                # data_log_all = [[INDEX data], [INDEX_M data], [THUMB data]]
                velocity_log_all = []
                position_log_all = []
                goal_position_log_all = []
                iq_log_all = []
                iq_raw_log_all = []
                vel_raw_log_all = []
                possibility_log_all = []
                contact_log_all = []

                for i in range(3):
                    velocity_log_all.append([data[i] for data in velocity_log])
                    vel_raw_log_all.append([data[i] for data in vel_raw_log])
                    position_log_all.append([data[i] for data in position_log])
                    goal_position_log_all.append([data[i] for data in goal_position_log])
                    iq_log_all.append([data[i] for data in iq_log])
                    iq_raw_log_all.append([data[i] for data in iq_raw_log])
                    possibility_log_all.append([data[i] for data in possibility_log])
                    contact_log_all.append([data[i] for data in contact_log])

                # # Plot here
                plt.figure(1)
                plt.subplot(311)
                plt.plot(time_log, position_log_all[0], 'b',
                         time_log, goal_position_log_all[0], 'c',
                         time_log, velocity_log_all[0], 'r',
                         time_log, contact_log_all[0], 'k')
                plt.grid(True)

                plt.subplot(312)
                plt.plot(time_log, position_log_all[1], 'b',
                         time_log, goal_position_log_all[1], 'c',
                         time_log, velocity_log_all[1], 'r',
                         time_log, contact_log_all[1], 'k')
                plt.grid(True)

                plt.subplot(313)
                plt.plot(time_log, position_log_all[2], 'b',
                         time_log, goal_position_log_all[1], 'c',
                         time_log, velocity_log_all[2], 'r',
                         time_log, contact_log_all[2], 'k')
                plt.grid(True)
                plt.show()
            pdb.set_trace()
            if error:
                self.MC.torque_enable_all(0)
                print("Grab error. System disabled.")
                return error
            else:
                # # Get contact iq for all fingers
                # self.contact_iq = [sum(data) / 20 for data in iq_window]
                # Run grab_end motion
                self.grab_end(logging=logging)


    def grab_end(self, logging=False):
        # Grab function ends up into this function
        # All BEARs take position command so that the present_iq reaches preload_iq+iq_compensation
        # In Parallel mode, THUMB takes 2*preload to balance load on object
        # Hold mode is different from Grip mode such that Hold mode is damping heavy

        error = 0
        finger_count = self.get_finger_count()
        # Change to force mode
        for idx in range(finger_count):
            self.MC.set_mode(self.robot.finger_ids[idx], 'force')
        # Check mode:
        if self.mode == 'H':
            # Hold mode, change to big D with small P
            # Calculate HOLD gains according to final_stiffness
            p_gain = round(HOLD_P_FACTOR * self.final_stiffness, 2)
            d_gain = min(round(HOLD_D_FACTOR * self.final_stiffness, 2), HOLD_D_MAX)
            print("Hold mode - p_gain: %2.2f, d_gain: %2.2f" % (p_gain, d_gain))
        else:
            # Grip mode
            # Calculate GRIP gains according to final_stiffness
            p_gain = round(GRIP_P_FACTOR * self.final_stiffness, 2)
            d_gain = round(GRIP_D_FACTOR * self.final_stiffness, 2)
            print("Grip mode - p_gain: %2.2f, d_gain: %2.2f" % (p_gain, d_gain))

        self.set_stiffness(finger_count, p_gain, d_gain)

        # Calculate preload iq to track
        signed_balance_factor = [(self.robot.fingerlist[idx].mirrored
                                  - (not self.robot.fingerlist[idx].mirrored)) * self.balance_factor[idx]
                                 for idx in range(finger_count)]

        # Get preload P and I gains
        preload_p, preload_i = self.preload_gains()

        # goal_iq = [self.preload * signed_balance_factor[idx] + self.contact_iq[idx] for idx in range(finger_count)]
        goal_iq = [self.preload * signed_balance_factor[idx]
                   + self.robot.iq_compensation[idx] for idx in range(finger_count)]

        # Initialize status variables
        iq = [0, 0, 0]
        prev_iq = [0, 0, 0]
        iq_error = [0, 0, 0]
        iq_error_int = [0, 0, 0]
        position = [0, 0, 0]
        goal_position = self.MC.get_present_position_all()  # populate with present pos
        goal_position_window = [[], [], []]

        # RAW(unfiltered data)
        iq_raw = [0, 0, 0]

        # iq window
        iq_window = [deque([0] * 20), deque([0] * 20), deque([0] * 20)]  # windowed queue for present iq of size 20

        # Status logging
        position_log = []
        goal_position_log = []
        time_log = []
        delta_time_log = []
        iq_log = []
        iq_error_log = []
        iq_raw_log = []

        delta_time = 0
        start_time = time.time()
        present_time = start_time
        time.sleep(0.001)  # Delay 1ms so that we don't get a zero delta_time for the first loop.

        delta_time_sum = 0
        loop_count = 0
        goal_iq_check = [False, False, False]
        goal_reached = False
        run = True
        # Track goal_iq
        print("Preloading...")
        while run and not error:
            # try:
            # get time stamp
            previous_time = present_time
            present_time = time.time()
            delta_time = present_time - previous_time

            # Collet status and send command
            status = self.MC.grab_loop_comm(self.robot.palm.gesture, goal_position, [0, 0, 0])

            # Process data
            # Motor Error
            motor_err = [i[1] for i in status]
            # Position
            position = [i[0][0] for i in status]
            # iq
            iq = [SMOOTHING_IQ * status[i][0][2] + (1 - SMOOTHING_IQ) * iq[i] for i in range(finger_count)]
            for idx in range(finger_count):
                iq_window[idx].append(iq[idx])
                iq_window[idx].popleft()

            # grab_end preload motion
            # Calculate preload_command
            # RobotController PID generating position command so that finger tracks goal_iq
            # Build commands
            iq_error = [goal_iq[i] - iq[i] for i in range(finger_count)]
            iq_error_int = [iq_error[i] * delta_time + iq_error_int[i] for i in range(finger_count)]

            # Loop to track goal_iq and confirm status with avrg(iq_window)
            # Once confirmed, keep running loop for 1 sec and maintain avrg(goal_pos)
            for idx in range(finger_count):
                # Build command
                goal_position[idx] = position[idx] + preload_p * iq_error[idx] + preload_i * iq_error_int[idx]
                if not goal_iq_check[idx]:
                    # This finger has not reached goal iq yet
                    avrg_iq = sum(iq_window[idx]) / 20
                    if abs(avrg_iq - goal_iq[idx]) < 0.05:
                        # Goal iq reached
                        goal_iq_check[idx] = True
                        print('Goal iq reached:', self.robot.fingerlist[idx].name)
            # bulk_read_write at loop head.
            if not goal_reached:
                goal_reached = sum(goal_iq_check) == finger_count
                retain_time_start = time.time()
            else:
                # All fingers have reached goal iq
                for idx in range(finger_count):
                    goal_position_window[idx].append(goal_position[idx])
                if time.time() - retain_time_start > 1:
                    # 1 sec has passed
                    run = False

            if logging:
                # Data logging
                time_log.append(present_time - start_time)
                iq_log.append(iq[:])
                iq_error_log.append(iq_error[:])
                position_log.append(position[:])
                goal_position_log.append(goal_position[:])

            # Check for timeout
            if present_time - start_time > TIMEOUT_GRIP:
                print("Grab end motion timeout, preload can not be reached")
                # self.MC.torque_enable_all(0)
                error = 1
            # except KeyboardInterrupt:
            #     print("User interrupted.")
            #     running = False
            #     error = 2
            time.sleep(0.001)

        # Out of while loop: preload reached or error
        # Data processing -logging
        if logging:
            # Format all data so that it is in this formation:
            # data_log_all = [[INDEX data], [INDEX_M data], [THUMB data]]
            position_log_all = []
            goal_position_log_all = []
            iq_log_all = []
            iq_error_log_all = []
            for i in range(3):
                position_log_all.append([data[i] for data in position_log])
                goal_position_log_all.append([data[i] for data in goal_position_log])
                iq_log_all.append([data[i] for data in iq_log])
                iq_error_log_all.append([data[i] for data in iq_error_log])

            # # Plot here
            plt.figure(2)
            plt.subplot(321)
            plt.plot(time_log, iq_log_all[0], 'b',
                     time_log, iq_error_log_all[0], 'c',
                     time_log, [goal_iq[0]] * len(iq_log_all[0]), 'r--')
            plt.grid(True)
            plt.subplot(322)
            plt.plot(time_log, position_log_all[0], 'b',
                     time_log, goal_position_log_all[0], 'c')
            plt.grid(True)

            plt.subplot(323)
            plt.plot(time_log, iq_log_all[1], 'b',
                     time_log, iq_error_log_all[1], 'c',
                     time_log, [goal_iq[1]] * len(iq_log_all[1]), 'r--')
            plt.grid(True)
            plt.subplot(324)
            plt.plot(time_log, position_log_all[1], 'b',
                     time_log, goal_position_log_all[1], 'c')
            plt.grid(True)

            plt.subplot(325)
            plt.plot(time_log, iq_log_all[2], 'b',
                     time_log, iq_error_log_all[2], 'c',
                     time_log, [goal_iq[2]] * len(iq_log_all[2]), 'r--')
            plt.grid(True)
            plt.subplot(326)
            plt.plot(time_log, position_log_all[2], 'b',
                     time_log, goal_position_log_all[2], 'c')
            plt.grid(True)
            plt.show()

        if error:
            self.MC.torque_enable_all(0)
            print("Grab end motion error. System disabled.")
            return error
        else:
            # Write average goal status
            goal_position = [sum(i) / len(i) for i in goal_position_window]
            self.MC.set_goal_position(self.robot.palm.gesture, goal_position)

    def release(self, release_mode, *hold_stiffness):
        """
        Release motion, all fingers run in position mode

        :param str release_mode: change-to-(H)old, (L)et-go, (F)ul-release
        :param float hold_stiffness: hold stiffness, only work for change to hold. (See Macros_DAnTE for default)
        """

        # TODO: if original stiffness is too low, it might not trigger full release end sign, thus use separate
        #  release PID

        # Check initialization
        if not self.robot.initialized:
            error = 3
            print("Robot not initialized. Exit.")
            return error

        # Check input
        if release_mode not in ['H', 'L', 'F']:
            print("Invalid mode.")  # TODO: Throw an exception
            return False

        if not self.get_robot_enable():
            # If robot is not enabled, enable it and only perform Full-release
            print("WARNING: Robot not enabled, enabling now.")
            self.set_robot_enable(1)

            if release_mode != 'F':
                print("WARNING: DAnTE can only perform full release in current status.")
                print("Switching to Full-release...")
                release_mode = 'F'

        elif not self.robot.contact:
            if release_mode != 'F':
                print("WARNING: DAnTE is currently not in contact with object thus can only perform full release.")
                print("Switching to Full-release...")
                release_mode = 'F'

        print("Releasing...")

        # Check mode:
        if release_mode == 'H':
            # Hold mode, change to big D with small P
            # Do not release contact
            # Enforced writing
            if len(hold_stiffness) == 0:
                # No hold_stiffness provided, go with default
                hold_stiffness = default_hold_stiffness
            # Calculate HOLD_D according to hold_stiffness
            hold_p = HOLD_P_FACTOR * hold_stiffness
            hold_d = HOLD_D_FACTOR * hold_stiffness
            if self.robot.palm.gesture == 'I':
                # Pinch mode, use only index fingers
                finger_count = 2
            else:
                # Use all three fingers
                finger_count = 3

            # Set D gain first
            for i in range(finger_count):
                self.MC.pbm.set_d_gain_force((self.robot.finger_ids[i], hold_d))
            # Enforce writing
            check = 0
            while check < finger_count:
                for i in range(finger_count):
                    if round(self.MC.pbm.get_d_gain_force(self.robot.finger_ids[i])[0][0][0], 2) != hold_d:
                        self.MC.pbm.set_d_gain_force((self.robot.finger_ids[i], hold_d))
                    else:
                        check += 1
            # Then set P gain
            for i in range(finger_count):
                self.MC.pbm.set_p_gain_force((self.robot.finger_ids[i], hold_p))
            # Enforce writing
            check = 0
            while check < finger_count:
                for i in range(finger_count):
                    if round(self.MC.pbm.get_p_gain_force(self.robot.finger_ids[i])[0][0][0], 2) != hold_p:
                        self.MC.pbm.set_p_gain_force((self.robot.finger_ids[i], hold_p))
                    else:
                        check += 1
            # Move goal_position forward for a bit more grabbing
            # Calculate goal_position
            goal_position = [
                round(self.contact_position[i] +
                      (self.robot.fingerlist[i].mirrored - (not self.robot.fingerlist[i].mirrored)) * delta_position, 4)
                for i in range(finger_count)]

            # Send command
            for i in range(finger_count):
                self.MC.pbm.set_goal_position((self.robot.finger_ids[i], goal_position[i]))
            # Enforce writing
            check = 0
            while check < finger_count:
                for i in range(finger_count):
                    if round(self.MC.pbm.get_goal_position(self.robot.finger_ids[i])[0][0][0], 4) != goal_position[i]:
                        self.MC.pbm.set_goal_position((self.robot.finger_ids[i], goal_position[i]))
                    else:
                        check += 1
            # Switch into Direct Force mode
            for i in range(finger_count):
                self.MC.set_mode(self.robot.finger_ids[i], 'force')

        else:
            # Let-go or Full-release
            # Uses existing mode and gains
            # Reset contact
            for f in self.robot.fingerlist:
                f.contact = False
            self.robot.contact = False

            if self.robot.palm.gesture == 'I':
                # Pinch mode, use only index fingers
                finger_count = 2
            else:
                # Use all three fingers
                finger_count = 3

            # Calculate goal_position
            if release_mode == 'L':
                # Let-go, just release a little
                sign = [(self.robot.fingerlist[i].mirrored - (not self.robot.fingerlist[i].mirrored)) for i in
                        range(finger_count)]
                goal_position = [self.contact_position[i] - sign[i] * let_go_margin for i in range(finger_count)]
            else:
                # Full-release
                goal_position = [0, 0, 0]

            # Send command
            for i in range(finger_count):
                self.MC.pbm.set_goal_position((self.robot.finger_ids[i], goal_position[i]))
            # Enforce writing
            check = 0
            while check < finger_count:
                for i in range(finger_count):
                    if round(self.MC.pbm.get_goal_position(self.robot.finger_ids[i])[0][0][0], 4) != goal_position[i]:
                        self.MC.pbm.set_goal_position((self.robot.finger_ids[i], goal_position[i]))
                    else:
                        check += 1

            # Make sure fingers stopped
            check = 0
            while check < finger_count:
                for i in range(finger_count):
                    if abs(self.MC.pbm.get_present_position(self.robot.finger_ids[i])[0][0][0]-goal_position[i]) < 0.3:
                        # This finger has come close enough to release point
                        check += 1
            print("Released.")
            # Put into IDLE if Full-release
            if release_mode == 'F':
                self.idle()

        time.sleep(0.5)

    # ------------------------
    # Utility Functions
    # ------------------------
    def read_initials(self):
        # Read initals file
        filename = 'Settings/initials.txt'
        filepath = os.path.join(str(Path(os.getcwd())), filename)
        if os.path.isfile(filepath):
            initials = open(filepath, 'r')
            data = initials.read()
            # String to num
            # Get rid of \n if any
            if data[-1] == '\n':
                data = data[:-1]
            # Convert to list of int and float
            init_data = []
            data = list(data.split("\n"))
            for data_string in data:
                data_list = list(data_string.split(","))
                data_list[0] = data_list[0][1:-1]
                data_list[1] = int(data_list[1])
                data_list[2:] = [float(i) for i in data_list[2:]]
                init_data.append(data_list)
            return init_data
        else:
            return None

    def read_iq_compensation(self):
        # Read iq_compensation file
        filename = 'Settings/iq_compensation.txt'
        filepath = os.path.join(str(Path(os.getcwd())), filename)
        if os.path.isfile(filepath):
            iq_comp_file = open(filepath, 'r')
            data = iq_comp_file.read()
            # String to num
            # Get rid of \n if any
            if data[-1] == '\n':
                data = data[:-1]
            if data[-1] == ',':
                data = data[:-1]
            # Convert to list of int and float
            data = list(data.split(","))
            iq_comp_data = [float(i) for i in data]
            return iq_comp_data
        else:
            return None

    def ping(self):
        """
        | Ping all motors
        |
        | Change PING_TRAIL_COUNT in Constants_DAnTE

        :return: ping error
        :rtype: int
        """
        # Ping all motors
        #
        error = 0b0000  # 4 bit respectively for INDEX, INDEX_M, THUMB, Dynamixel
        # BEAR:
        for idx, f in enumerate(self.robot.fingerlist):
            trail = 0
            check = False
            print("Pinging %s..." % f.name)
            while not check:
                if not bool(self.MC.pbm.ping(f.motor_id)[0]):
                    trail += 1
                    if trail > PING_TRAIL_COUNT:
                        # Tried PING_TRAIL_COUNT times and still no luck:
                        print("ERROR: %s offline." % f.name)
                        error = error | (1 << idx)
                        break
                    # Retry in 0.5s
                    # print("Retry in 0.5 second.")
                    time.sleep(0.5)
                else:
                    # Ping succeed
                    check = True
                    if trail > int(PING_TRAIL_COUNT / 2):
                        # WARNING about bad communication
                        print("WARNING: %s BEAR communication intermittent." % f.name)
        # DXL:
        if not self.bypass_DXL:
            trail = 0
            check = False
            print("Pinging Palm actuator...")
            while not check:
                if not self.DC.ping():
                    trail += 1
                    if trail > PING_TRAIL_COUNT:
                        # Tried PING_TRAIL_COUNT times and still no luck:
                        print("ERROR: Palm actuator offline.")
                        error = error | (1 << 3)
                        break
                    # Retry in 0.5s
                    # print("Retry in 0.5 second.")
                    time.sleep(0.5)
                else:
                    # Ping succeed
                    check = True
                    if trail > int(PING_TRAIL_COUNT / 2):
                        # WARNING about bad communication
                        print("WARNING: Palm actuator communication intermittent.")
        return error

    def get_robot_enable(self):
        """
        See if all motors are enabled
        :return: bool
        """
        if self.bypass_DXL:
            enable = sum(self.MC.get_enable_all()) == 3
        else:
            enable = (sum(self.MC.get_enable_all()) + self.DC.get_enable()) == 4
        return enable

    def set_robot_enable(self, val):
        """
        Enable/disable the robot
        :param val: 0: disable, 1: enable
        :type val: int
        """
        # Enable/disable all actuators
        if self.bypass_DXL:
            self.MC.torque_enable_all(val)
        else:
            self.MC.torque_enable_all(val)
            self.DC.torque_enable(val)

    def idle(self, *fingers):
        # Put specified finger into idle.
        # If no finger specified, put whole hand in IDLE
        # Dynamixel hold current position
        # Related BEAR enters position mode and go to home with IDLE gains.
        if not self.bypass_DXL:
            palm_pos = self.DC.get_present_position()
            self.DC.set_goal_position(palm_pos)

        if len(fingers) == 0:
            fingers = self.robot.fingerlist
        for f in fingers:
            self.MC.pbm.set_bulk_config((f.motor_id, 'p_gain_position', IDLE_P,
                                         'i_gain_position', IDLE_I,
                                         'd_gain_position', IDLE_D))
            self.MC.set_mode(f.motor_id, 'position')
            self.MC.torque_enable(f.motor_id, 1)
            self.MC.pbm.set_goal_position((f.motor_id, 0))
            print("%s in idle." % f.name)

    def get_finger_count(self):
        # Get finger_count from gesture for stiffness and force balancing purposes.
        # This function is used before set_stiffness and all force commands.
        if self.robot.palm.gesture == 'I':
            # Pinch mode, use only index fingers
            finger_count = 2
        elif self.robot.palm.gesture == 'Y' or 'P':
            # Use all three fingers
            finger_count = 3
        else:
            print("Invalid gesture status.")  # TODO: Throw exception
            return False
        return finger_count

    def set_stiffness(self, finger_count, p_gain, d_gain):
        """
        Set the P,D gains for Direct Force mode with enforced writing

        :param int finger_count: number of fingers generated by get_finger_count()
        :param float p_gain: BEAR p_gain
        :param float d_gain: BEAR d_gain
        :return:
        """
        print("Setting stiffness")
        # Set D gain first
        for i in range(finger_count):
            self.MC.pbm.set_d_gain_force((self.robot.finger_ids[i], d_gain))
        # Enforce writing
        check = 0
        while check < finger_count:
            for i in range(finger_count):
                if round(self.MC.pbm.get_d_gain_force(self.robot.finger_ids[i])[0][0][0], 2) != round(d_gain, 2):
                    self.MC.pbm.set_d_gain_force((self.robot.finger_ids[i], d_gain))
                else:
                    check += 1
        # Then set P gain
        for i in range(finger_count):
            self.MC.pbm.set_p_gain_force((self.robot.finger_ids[i], p_gain))
        # Enforce writing
        check = 0
        while check < finger_count:
            for i in range(finger_count):
                if round(self.MC.pbm.get_p_gain_force(self.robot.finger_ids[i])[0][0][0], 2) != round(p_gain, 2):
                    self.MC.pbm.set_p_gain_force((self.robot.finger_ids[i], p_gain))
                else:
                    check += 1
        print("Stiffness set.")
        return True

    def contact_possibility(self, iq, vel, t):
        """
        Calculate contact possibility
        Function:
        possibility = 10t*(1/(a^iq) -1) /velocity
        a is a factor, the samller it is, the more significant effect iq has.
        t is total time since start and is no bigger than 0.1
        """
        factor_a = 0.25
        iq = min(iq, 1)
        vel = max(vel, 0.3)
        p = min((10 * min(t, 0.1) / (factor_a ** iq) - 1) / vel, 10) / 10
        return p

    def contact_possibility_criteria(self):
        """
        Optimal result from experiments
        """
        if self.approach_speed > 1:
            lower_threshold = 0.1
            confident_threshold = 0.2
        elif self.approach_speed < 0.5:
            lower_threshold = 0.3
            confident_threshold = 0.35
        else:
            lower_threshold = -0.8*(self.approach_speed-0.5)**2 + 0.3
            confident_threshold = -0.6*(self.approach_speed-0.5)**2 + 0.35
        integration_threshold = lower_threshold*4

        return [lower_threshold, confident_threshold, integration_threshold]


    def preload_gains(self):
        # Optimal preload p and i gains are related to gains on BEAR
        if self.mode == 'H':
            # Hold mode
            p_gain = 5/self.final_stiffness
            i_gain = 20/self.final_stiffness
        else:
            # Grip mode
            p_gain = 0.5 / self.final_stiffness
            i_gain = 10 / self.final_stiffness
        return p_gain, i_gain

    # ------------------------
    # Forward Kinematics
    # ------------------------

    def update_angles(self):
        """Calculate and update all joint angles in finger.angles"""
        error = 0  # 1 for timeout, 2 for user interruption, 3 for initialization, 9 for Invalid input
        # Check initialization
        if not self.robot.initialized:
            error = 3
            print("Robot not initialized. Exit.")
            return error
        # Update finger joint angles
        present_pos = self.MC.get_present_position_all()  # [Index, Index_M, THUMB]
        # Get ext_enc reading:
        self.ext_enc.connect()
        time.sleep(0.2)
        ext_reading = self.ext_enc.get_angle()
        self.ext_enc.release()
        # Update all joint angles
        for idx, finger in enumerate(self.robot.fingerlist):
            finger.angles[0] = alpha_0[idx] + present_pos[idx]  # Get alpha from present position

            finger.angles[1] = ext_reading[idx] - finger.encoder_offset + math.pi / 6  # Get beta from external encoders
            # Get [gamma, delta] from present position
            finger.angles[2:4] = FK.angles(finger, self.robot.palm.angle)

            print(finger.name)
            # print("alpha_0: %f" % alpha_0[idx])
            bear_pos = present_pos[idx] * 180 / math.pi
            print("present_pos: %f" % bear_pos)
            # print("alpha: %f" % finger.angles[0])
            angles_in_deg = [i * 180 / math.pi for i in finger.angles]
            print(angles_in_deg)
        print("Palm: " + str(self.robot.palm.angle))

    def forward_kinematics(self, *finger):
        """
        Calculate finger forward kinematics and return joint locations.
        Updates finger.joint_locations

        :param object finger: Specify the finger to be calculated, otherwise all fingers will be calculated
        """
        # Specify the finger to be calculated, otherwise all fingers will be calculated
        error = 0  # 1 for timeout, 2 for user interruption, 3 for initialization, 9 for Invalid input
        # Check initialization
        if not self.robot.initialized:
            error = 3
            print("Robot not initialized. Exit.")
            return error
        palm_angle = self.robot.palm.angle
        if len(finger) == 0:
            # update all finger kinematics
            for f in self.robot.fingerlist:
                FK.finger_fk(f, palm_angle)
        else:
            FK.finger_fk(finger, palm_angle)

    def visualization(self):
        error = 0  # 1 for timeout, 2 for user interruption, 3 for initialization, 9 for Invalid input
        # Check initialization
        if not self.robot.initialized:
            error = 3
            print("Robot not initialized. Exit.")
            return error
        FK.visual(self.robot)
