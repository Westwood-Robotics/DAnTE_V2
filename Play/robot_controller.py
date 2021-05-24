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
import matplotlib.pyplot as plt
import math
from collections import deque

if EXTERNAL_ENC:
    # Only import the following when EXTERNAL_ENC is True as wiringpi and spidev are required.
    import Forward_Kinematics.forward_kin as FK
    from Play.MPS import MPS_Encoder_Cluster

import pdb


# -----------------------------
# Functions for basic robot motions
# Enable all before sending commands


def read_initials():
    # Read initals file
    filename = 'Settings/initials.txt'
    filepath = os.path.join(str(Path(os.getcwd())), filename)
    # filepath = os.path.join(str(Path(os.getcwd()).parent), filename)
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
        self.contact_iq = [0, 0, 0]
        self.balance_factor = [1, 1, 1]  # Factor for force balance between fingers, update when change gesture

        self.welcome_msg()

        # self.start_robot()

    def welcome_msg(self):
        print("=========== DAnTE version 2.0.0 -- Last Updated 2020.06.24 ===")
        print("==============================================================")

    # ------------------------
    # INITIALIZATION FUNCTIONS
    # ------------------------
    # Functions for initialization of finger(s)
    # Read initials.txt and check settings
    # Move finger(s) through range of motion according to initials.txt and check for mobility and interference
    def start_robot(self):

        error = 0b0000  # 4 bit respectively for INDEX, INDEX_M, THUMB, Dynamixel, 0b10000 for overall error.

        # Ping all motors
        # Specify PING_TRAIL_COUNT in Constants_DAnTE
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

        # Read initials, fact check and populate robot object
        init_data = read_initials()  # init_data = [['FINGER', motor_id, homing_offset, travel, encoder_offset]...]
        if not self.bypass_DXL and len(init_data) < 4:
            print("Length of init_data is too short. Exiting...")
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

    def get_robot_enable(self):
        """
        See if all motors are enabled
        :return: bool
        """
        if self.bypass_DXL:
            enable = sum(self.MC.get_enable_all()) == 3
        else:
            enable = (sum(self.MC.get_enable_all())+self.DC.get_enable()) == 4
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
                if (self.robot.palm.home-0.2) < palm_pos < (self.robot.palm.home+1.77):
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

        # Ask user if abnoraml
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
        # pdb.set_trace()
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
                    ext_enc_error = True
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
            return True

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
                    DXL_goal_pos = self.robot.palm.home+math.pi/3
                    self.robot.palm.angle = math.pi/3
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

    def grab_old(self, gesture, mode, **options):
        """
        Control grab motion of self.robot
        If keyword options not specified, it will go with a default. See Macros_DAnTE

        :param str gesture: Grab with a gesture, tripod(Y), pinch(I) or parallel(P)
        :param str mode: Specify a grab mode: (H)old or (G)rip

        :keyword float approach_speed: Approach speed
        :keyword float approach_stiffness: Approach stiffness
        :keyword float detect_current: Detection current
        :keyword float final_stiffness: Grip/Hold stiffness
        :keyword float preload: Grip preload, preload in Amps when gripping, no effect on Hold.
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
            self.detect_current = max(options.get("detect_current", default_detect_current), detect_current_min)
            self.final_stiffness = options.get("final_stiffness", default_final_stiffness)
            self.preload = options.get("preload", default_preload)
            self.max_iq = max(options.get("max_iq", default_max_iq), self.detect_current, default_max_iq)
            logging = options.get("logging", False)

        # Calculate approach_i from approach_stiffness
        approach_i = approach_i_func(self.approach_stiffness)
        # Set detect_count if detect_current is below confident value
        if self.detect_current < confident_detect_current:
            detect_count = [detect_confirm, detect_confirm, detect_confirm]
        else:
            detect_count = [0, 0, 0]

        contact_count = 0

        # Calculate goal_approach_speed
        goal_approach_speed = [-self.approach_speed + 2 * self.approach_speed * f.mirrored for f in
                               self.robot.fingerlist]

        # Start with change into gesture
        self.change_gesture(gesture)
        # Set fingers' Direct Force PID according to Stiffness
        # self.set_approach_stiffness()
        force_p = self.approach_stiffness
        force_d = 0.1 * force_p
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
            acceleration = [0, 0, 0]
            iq = [0, 0, 0]
            iq_comp = [0, 0, 0]
            goal_iq = [0, 0, 0]
            position = [0, 0, 0]
            goal_position = self.MC.get_present_position_all() # populate with present pos

            # Status logging
            velocity_log = []
            position_log = []
            time_log = []
            delta_time_log = []
            iq_log = []
            iq_comp_log = []
            acceleration_log = []

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
                    delta_time = present_time-previous_time

                    # Collect status and send command
                    status = self.MC.grab_loop_comm(self.robot.palm.gesture, goal_position, goal_iq)

                    # print(time.time()-present_time)

                    # Process data
                    # Motor Error
                    motor_err = [i[1] for i in status]
                    # Position
                    position = [i[0][0] for i in status]
                    # iq
                    iq = [SMOOTHING * status[i][0][2] + (1 - SMOOTHING) * iq[i] for i in range(finger_count)]
                    # Velocity
                    prev_velocity = velocity
                    velocity = [SMOOTHING * status[i][0][1] + (1 - SMOOTHING) * velocity[i] for i in
                                range(finger_count)]

                    # Acceleration
                    acceleration = [
                        SMOOTHING * (velocity[i] - prev_velocity[i]) / delta_time + (1 - SMOOTHING) * acceleration[i]
                        for i in range(finger_count)]
                    # Get compensated iq
                    iq_comp = [abs(abs(iq[i] - acceleration[i] * ACC_COMP_FACTOR)
                                   - (abs(position[i]) < SPRING_COMP_START) * 0.2
                                   - (abs(position[i]) > SPRING_COMP_START) * (
                                               0.18 + 0.06 * (abs(position[i]) - SPRING_COMP_START)))
                               for i in range(finger_count)]

                    # Approach motion
                    # Calculate approach_command
                    # RobotController PID generating position command so that finger tracks approach_speed
                    # Build approach commands
                    velocity_error = [goal_approach_speed[i] - velocity[i] for i in range(finger_count)]
                    velocity_error_int = [velocity_error[i] * delta_time + velocity_error_int[i] for i in range(finger_count)]

                    # Determine if contact and maintain position upon contact
                    # Calculate goal_position before contact
                    for idx in range(finger_count):
                        if not self.robot.fingerlist[idx].contact:
                            if iq_comp[idx] > self.detect_current:
                                if detect_count[idx]:
                                    detect_count[idx] -= 1
                                    # Build command
                                    goal_position[idx] = position[idx] \
                                                         + approach_p * velocity_error[idx] \
                                                         + approach_i * velocity_error_int[idx] \
                                                         - approach_d * acceleration[idx]
                                else:
                                    self.robot.fingerlist[idx].contact = True
                                    print('Finger contact:', self.robot.fingerlist[idx].name, position[idx])
                                    self.contact_position[idx] = position[idx]
                                    # Calculate iq to maintain
                                    # Get sign and balance factor for the finger
                                    iq_sign_balance = (self.robot.fingerlist[idx].mirrored - (not self.robot.fingerlist[idx].mirrored))*self.balance_factor[idx]
                                    goal_iq[idx] = iq_sign_balance*(self.detect_current + (abs(position[idx]) > SPRING_COMP_START) * (0.18 + 0.06 * (abs(position[idx]) - SPRING_COMP_START)))
                                    # Send goal_iq
                                    print('Finger iq:', goal_iq[idx])
                                    # Set into torque mode
                                    self.MC.set_mode(self.robot.finger_ids[idx], 'torque')
                                    contact_count += 1
                            else:
                                # Build command
                                goal_position[idx] = position[idx] + approach_p * velocity_error[idx] + approach_i * \
                                                     velocity_error_int[idx] - approach_d * acceleration[idx]
                        else:
                            # This finger has contacted
                            # Keep sending iq command
                            pass
                    # bulk_read_write at loop head.

                    self.robot.contact = contact_count == finger_count

                    # # Clamp approach_command
                    # approach_command = [max(min(i, approach_command_max), -approach_command_max) for i in approach_command]

                    # Data logging
                    if logging:
                        delta_time_log.append(delta_time)
                        time_log.append(present_time - start_time)
                        velocity_log.append(velocity)
                        position_log.append(position)
                        iq_log.append(iq)
                        iq_comp_log.append(iq_comp)
                        acceleration_log.append(acceleration)

                    # Check for timeout
                    if present_time - start_time > TIMEOUT_GRAB:
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
            avg_loop_time = delta_time_sum/loop_count
            print("Average loop time: %f" % avg_loop_time)
            # Data processing -logging
            if logging:
                # Format all data so that it is in this formation:
                # data_log_all = [[INDEX data], [INDEX_M data], [THUMB data]]
                velocity_log_all = []
                position_log_all = []
                iq_log_all = []
                iq_comp_log_all = []
                acceleration_log_all = []

                for i in range(3):
                    velocity_log_all.append([data[i] for data in velocity_log])
                    position_log_all.append([data[i] for data in position_log])
                    iq_log_all.append([data[i] for data in iq_log])
                    iq_comp_log_all.append([data[i] for data in iq_comp_log])
                    acceleration_log_all.append([data[i] for data in acceleration_log])

                # # Plot here
                # id = 0
                # plt.plot(time_log, iq_comp_log_all[0], 'r-', time_log, iq_comp_log_all[1], 'k-', time_log, iq_comp_log_all[2], 'b-')
                # # plt.plot(time_log, delta_time_log)
                # plt.grid(True)
                # plt.show()
            if error:
                self.MC.torque_enable_all(0)
                print("Grab error. System disabled.")
            else:
                # # Debug option
                # # Get current status for debug purpose.
                # # Get mode for all
                # modes = self.MC.pbm.get_mode(1, 2, 3)
                # modes = [i[0] for i in modes]
                # print('Modes:', modes)
                # # Get goal_iq for all
                # g_iq = self.MC.pbm.get_goal_iq(1, 2, 3)
                # g_iq = [i[0] for i in g_iq]
                # print('Goal_iq:', g_iq)
                # # Get present_iq for all
                # p_iq = self.MC.pbm.get_present_iq(1, 2, 3)
                # p_iq = [i[0] for i in p_iq]
                # print('Present_iq:', p_iq)

                # Switch to final grip/hold upon object detection
                # pdb.set_trace()
                # usr = input("Press enter to proceed to grab_end...")

                self.grab_end()

        return error

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
            self.max_iq = max(options.get("max_iq", default_max_iq), default_max_iq)
            logging = options.get("logging", False)

            #update approach stiffness
            self.approach_stiffness = approach_stiffness_func(self.approach_speed, self.approach_stiffness)

        # Calculate approach_i from approach_stiffness
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
            possibility_int = [0, 0, 0] # integration of relatively low possibility


            # RAW(unfiltered data)
            vel_raw = [0, 0, 0]
            iq_raw = [0, 0, 0]

            # iq window
            iq_window = [deque([0]*20), deque([0]*20), deque([0]*20)]  # windowed queue for present iq of size 20

            # Status logging
            velocity_log = []
            position_log = []
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

            # Use below to compensate iq for more reliable possibility
            iq_compensation = [-0.18720547876157367, 0.14256043923745057, -0.3072237240286084]
            # TODO: Above should be imported from file

            while not (error or self.robot.contact):
                # Main GRAB loop
                try:
                    # Get time stamp
                    previous_time = present_time
                    present_time = time.time()
                    delta_time = present_time - previous_time
                    grab_time = present_time - start_time

                    # Collect status and send command
                    status = self.MC.grab_loop_comm(self.robot.palm.gesture, goal_position)

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

                    # Determine if contact and Switch to torque mode and maintain detect iq upon contact
                    # Calculate goal_iq/goal_position and switch mode for contacted
                    # Just send both goal commands, let BEAR choose which to use.
                    for idx in range(finger_count):
                        if not self.robot.fingerlist[idx].contact:
                            # Keep it running for now
                            # Get possibility
                            possibility[idx] = self.contact_possibility(abs(iq[idx]-iq_compensation[idx]),
                                                                        abs(velocity[idx]),
                                                                        grab_time)
                            # TODO: Higher speed can use lower possibility
                            if possibility[idx]>0.3:
                                possibility_int[idx] += possibility[idx]
                            if possibility[idx]>0.4 or possibility_int[idx]>1.5:
                                # This finger has come to contact
                                self.robot.fingerlist[idx].contact = True
                                print('Finger contact:', self.robot.fingerlist[idx].name, 'Position:', position[idx])
                                self.contact_position[idx] = position[idx]
                                goal_position[idx] = self.contact_position[idx]
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
            self.MC.set_goal_position(self.robot.palm.gesture, goal_position)

            avg_loop_time = delta_time_sum / loop_count
            print("Average loop time: %f" % avg_loop_time)

            # Data processing -logging
            if logging:
                # Format all data so that it is in this formation:
                # data_log_all = [[INDEX data], [INDEX_M data], [THUMB data]]
                velocity_log_all = []
                position_log_all = []
                iq_log_all = []
                iq_raw_log_all = []
                vel_raw_log_all = []
                possibility_log_all = []
                contact_log_all = []

                for i in range(3):
                    velocity_log_all.append([data[i] for data in velocity_log])
                    vel_raw_log_all.append([data[i] for data in vel_raw_log])
                    position_log_all.append([data[i] for data in position_log])
                    iq_log_all.append([data[i] for data in iq_log])
                    iq_raw_log_all.append([data[i] for data in iq_raw_log])
                    possibility_log_all.append([data[i] for data in possibility_log])
                    contact_log_all.append([data[i] for data in contact_log])

                # # Plot here
                plt.figure(1)
                plt.subplot(311)
                plt.plot(#time_log, vel_raw_log_all[0], 'b',
                         time_log, velocity_log_all[0], 'c',
                         time_log, iq_log_all[0], 'r',
                         #time_log, iq_raw_log_all[0], 'm',
                         time_log, possibility_log_all[0], 'g',
                         time_log, contact_log_all[0], 'k')
                plt.grid(True)

                plt.subplot(312)
                plt.plot(time_log, vel_raw_log_all[1], 'b',
                         #time_log, velocity_log_all[1], 'c',
                         time_log, iq_log_all[1], 'r',
                         #time_log, iq_raw_log_all[1], 'm',
                         time_log, possibility_log_all[1], 'g',
                         time_log, contact_log_all[1], 'k')
                plt.grid(True)

                plt.subplot(313)
                plt.plot(#time_log, vel_raw_log_all[2], 'b',
                         time_log, velocity_log_all[2], 'c',
                         time_log, iq_log_all[2], 'r',
                         #time_log, iq_raw_log_all[2], 'm',
                         time_log, possibility_log_all[2], 'g',
                         time_log, contact_log_all[2], 'k')
                plt.grid(True)
                plt.show()

                pdb.set_trace()

            if error:
                self.MC.torque_enable_all(0)
                print("Grab error. System disabled.")
                return error
            else:
                # Get contact iq for all fingers
                self.contact_iq = [sum(data) / 20 for data in iq_window]
                # Run grab_end motion
                self.grab_end(logging=True)

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
        p = min((10*min(t,0.1)/(factor_a**iq)-1)/vel, 10)/10
        return p

    def get_contact_possibility_criteria(self):
        # TODO: The contact possibility criteria is lower when approach speed is high, while higher when approach
        #  speed is low
        self.approach_speed

    def grab_end_old(self, plot=False):

        error = 0
        finger_count = self.get_finger_count()
        # Check mode:
        if self.mode == 'H':
            # Hold mode, change to big D with small P

            # Calculate HOLD gains according to final_stiffness
            p_gain = round(HOLD_P_FACTOR * self.final_stiffness, 2)
            d_gain = round(HOLD_D_FACTOR * self.final_stiffness, 2)
            # if self.robot.palm.gesture == 'I':
            #     # Pinch mode, use only index fingers
            #     finger_count = 2
            # else:
            #     # Use all three fingers
            #     finger_count = 3
            #
            # # Set D gain first
            # for i in range(finger_count):
            #     self.MC.pbm.set_d_gain_force((self.robot.finger_ids[i], hold_d))
            # # Enforce writing
            # check = 0
            # while check < finger_count:
            #     for i in range(finger_count):
            #         if round(self.MC.pbm.get_d_gain_force(self.robot.finger_ids[i])[0][0], 2) != hold_d:
            #             self.MC.pbm.set_d_gain_force((self.robot.finger_ids[i], hold_d))
            #         else:
            #             check += 1
            # # Then set P gain
            # for i in range(finger_count):
            #     self.MC.pbm.set_p_gain_force((self.robot.finger_ids[i], hold_p))
            # # Enforce writing
            # check = 0
            # while check < finger_count:
            #     for i in range(finger_count):
            #         if round(self.MC.pbm.get_p_gain_force(self.robot.finger_ids[i])[0][0], 2) != hold_p:
            #             self.MC.pbm.set_p_gain_force((self.robot.finger_ids[i], hold_p))
            #         else:
            #             check += 1

            self.set_stiffness(finger_count, p_gain, d_gain)

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
            # Grip mode, grip to preload

            # Calculate GRIP gains according to final_stiffness
            p_gain = round(GRIP_P_FACTOR * self.final_stiffness, 2)
            d_gain = round(GRIP_D_FACTOR * self.final_stiffness, 2)
            self.set_stiffness(finger_count, p_gain, d_gain)

            detect_count = [grip_confirm, grip_confirm, grip_confirm]
            iq_comp_goal = self.preload

            if self.robot.palm.gesture == 'I':
                # Pinch mode, use only index fingers
                finger_count = 2
            else:
                # Use all three fingers
                finger_count = 3

            # Get goal_iq
            goal_iq = [
                (self.robot.fingerlist[i].mirrored - (not self.robot.fingerlist[i].mirrored)) *
                (iq_comp_goal + (abs(self.contact_position[i]) < SPRING_COMP_START) * 0.2 +
                 (abs(self.contact_position[i]) > SPRING_COMP_START) * (0.18 + 0.06 * (abs(self.contact_position[i]) - SPRING_COMP_START)))
                for i in range(finger_count)]
            # Clamp goal_iq with max_iq and balance force
            goal_iq = [min(max(goal_iq[i], -self.max_iq), self.max_iq)*self.balance_factor[i] for i in range(finger_count)]

            print("Grip goal_iq:", goal_iq)

            iq = [0, 0, 0]
            iq_error_int = [0, 0, 0]

            goal_reached = False
            goal_reached_count = 0
            finger_goal_reached = [False, False, False]
            start_time = time.time()
            present_time = time.time()

            first_cyc = True

            # Logging initialize
            time_log = []
            iq_log = []
            position_log = []
            grip_command_log = []
            iq_error_log = []

            # Track goal_iq
            while not (error or goal_reached):
                try:
                    previous_time = present_time

                    # Collect status
                    if self.robot.palm.gesture == 'I':
                        status = self.MC.get_present_status_index()
                    else:
                        status = self.MC.get_present_status_all()

                    # Get time stamp
                    present_time = time.time()
                    delta_time = present_time - previous_time

                    # Process data
                    # Motor Error
                    motor_err = [i[1] for i in status]
                    # iq
                    iq = [SMOOTHING * status[i][0][2] + (1 - SMOOTHING) * iq[i] for i in range(finger_count)]

                    iq_error = [goal_iq[i] - iq[i] for i in range(finger_count)]

                    # Determine if goal_reached
                    for idx in range(finger_count):
                        if finger_goal_reached[idx]:
                            pass
                        else:
                            if abs(iq_error[idx]) < 0.1:
                                if detect_count[idx]:
                                    detect_count[idx] -= 1
                                else:
                                    finger_goal_reached[idx] = True
                                    goal_reached_count += 1
                    goal_reached = goal_reached_count == finger_count

                    # Grip motion
                    # Calculate goal_position
                    # RobotController PID generating position command so that finger tracks goal_iq
                    iq_error_int = [iq_error[i] * delta_time + iq_error_int[i] for i in range(finger_count)]

                    # Build command, stop finger upon contact
                    grip_command = [self.contact_position[i] +
                                    (grip_p * iq_error[i] + grip_i * iq_error_int[i])
                                    for i in range(finger_count)]

                    if finger_count == 2:
                        # Pinch mode, only use INDEX fingers
                        self.MC.pbm.set_goal_position((BEAR_INDEX, grip_command[0]),
                                                      (BEAR_INDEX_M, grip_command[1]))

                    else:
                        # Use all fingers
                        self.MC.pbm.set_goal_position((BEAR_INDEX, grip_command[0]),
                                                      (BEAR_INDEX_M, grip_command[1]),
                                                      (BEAR_THUMB, grip_command[2]))
                    if first_cyc:
                        # Switch into Direct Force mode
                        for i in range(finger_count):
                            self.MC.set_mode(self.robot.finger_ids[i], 'force')
                        first_cyc = False

                    # Data logging
                    time_log.append(present_time-start_time)
                    iq_log.append(iq)
                    grip_command_log.append(grip_command)
                    iq_error_log.append(iq_error)

                    # Check for timeout
                    if present_time - start_time > TIMEOUT_GRIP:
                        print("GRIP motion timeout, preload can not be reached")
                        # self.MC.torque_enable_all(0)
                        error = 1
                except KeyboardInterrupt:
                    print("User interrupted.")
                    running = False
                    error = 2

            # Out of while loop, preload reached or error

            if plot:
                # Plot functions
                iq_log_all = []
                grip_command_log_all = []
                iq_error_log_all = []

                for i in range(finger_count):
                    iq_log_all.append([data[i] for data in iq_log])
                    iq_error_log_all.append([data[i] for data in iq_error_log])
                    grip_command_log_all.append([data[i] for data in grip_command_log])

                # # Plot here
                # id = 2
                # # plt.plot(time_log, iq_comp_log_all[0], 'r-', time_log, iq_comp_log_all[1], 'k-', time_log, iq_comp_log_all[2], 'b-')
                # plt.plot(time_log, iq_log_all[0], 'r-',
                #          time_log, iq_log_all[1], 'b-',
                #          time_log, iq_log_all[2], 'g-')
                # plt.grid(True)
                # plt.show()

        # TODO: Check error

    def grab_end(self, logging=False):
        # Grab function ends up into this function
        # All BEARs take posision command so that the present_iq reaches preload_iq+contact_iq
        # In Parallel mode, THUMB takes 2*preload to balance load on object
        # Hold mode is different from Grip mode such that Hold mode is damping heavy

        error = 0
        finger_count = self.get_finger_count()
        # Check mode:
        if self.mode == 'H':
            # Hold mode, change to big D with small P
            # Calculate HOLD gains according to final_stiffness
            p_gain = round(HOLD_P_FACTOR * self.final_stiffness, 2)
            d_gain = min(round(HOLD_D_FACTOR * self.final_stiffness, 2), HOLD_D_MAX)
            print("Hold mode - p_gain: %2.2f, d_gain: %2.2f"% (p_gain, d_gain))
        else:
            # Grip mode
            # Calculate GRIP gains according to final_stiffness
            p_gain = round(GRIP_P_FACTOR * self.final_stiffness, 2)
            d_gain = round(GRIP_D_FACTOR * self.final_stiffness, 2)
            print("Grip mode - p_gain: %2.2f, d_gain: %2.2f" % (p_gain, d_gain))

        self.set_stiffness(finger_count, p_gain, d_gain)

        # Calculate preload iq to track
        signed_balance_factor = [(self.robot.fingerlist[idx].mirrored
                                 - (not self.robot.fingerlist[idx].mirrored))*self.balance_factor[idx]
                                 for idx in range(finger_count)]

        goal_iq = [self.preload*signed_balance_factor[idx]+self.contact_iq[idx] for idx in range(finger_count)]

        # Initialize status variables
        iq = [0, 0, 0]
        prev_iq = [0, 0, 0]
        iq_error = [0, 0, 0]
        iq_error_int = [0, 0, 0]
        position = [0, 0, 0]
        goal_position = self.MC.get_present_position_all()  # populate with present pos
        goal_position_window = [[],[],[]]

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
            status = self.MC.grab_loop_comm(self.robot.palm.gesture, goal_position)

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
                    avrg_iq = sum(iq_window[idx])/20
                    if abs(avrg_iq - goal_iq[idx])<0.05:
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
                if time.time()-retain_time_start > 1:
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
            plt.figure(1)
            plt.subplot(321)
            plt.plot(time_log, iq_log_all[0], 'b',
                     time_log, iq_error_log_all[0], 'c',
                     time_log, [goal_iq[0]]*len(iq_log_all[0]), 'r--')
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

            pdb.set_trace()

        if error:
            self.MC.torque_enable_all(0)
            print("Grab end motion error. System disabled.")
            return error
        else:
            # Write average goal status
            goal_position = [sum(i)/len(i) for i in goal_position_window]
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
                sign = [(self.robot.fingerlist[i].mirrored - (not self.robot.fingerlist[i].mirrored)) for i in range(finger_count)]
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
                    if abs(self.MC.pbm.get_present_position(self.robot.finger_ids[i])[0][0][0] - goal_position[i])<0.3:
                        # This finger has come close enough to release point
                        check += 1
            print("Released.")
            # Put into IDLE if Full-release
            if release_mode == 'F':
                self.idle()

        time.sleep(0.5)

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

            finger.angles[1] = ext_reading[idx] - finger.encoder_offset + math.pi/6  # Get beta from external encoders
            # Get [gamma, delta] from present position
            finger.angles[2:4] = FK.angles(finger, self.robot.palm.angle)

            print(finger.name)
            # print("alpha_0: %f" % alpha_0[idx])
            bear_pos = present_pos[idx]*180/math.pi
            print("present_pos: %f" % bear_pos)
            # print("alpha: %f" % finger.angles[0])
            angles_in_deg = [i*180/math.pi for i in finger.angles]
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


if __name__ == '__main__':
    rc = RobotController(robot=DAnTE)
    rc.start_robot()
    rc.initialization()
    rc.grab('P', 'H', approach_speed=1.8, approach_stiffness=0.35, detect_current=0.3, max_iq=0.25, final_strength=2)