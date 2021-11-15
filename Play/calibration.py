#!usr/bin/env python
__author__ = "Xiaoguang Zhang"
__email__ = "xzhang@westwoodrobotics.net"
__copyright__ = "Copyright 2020 Westwood Robotics"
__date__ = "Jan 8, 2020"
__version__ = "0.1.0"
__status__ = "Beta"

import time
import os
import numpy as nmp
from pathlib import Path
from Play.motor_controller import MotorController
from Play.dynamixel_controller import DynamixelController
from Settings.Robot import *
from Settings.Constants_DAnTE import *

if EXTERNAL_ENC:
    # Only import the following when EXTERNAL_ENC is True as wiringpi and spidev are required.
    from Play.MPS import MPS_Encoder_Cluster

import pdb


def calibration_geometry(robot, bypass_DXL=False, bypass_ext_enc=False):
    # Calibrate the whole hand.

    # When debug, you might want to bypass Dynamixel
    if bypass_DXL:
        DXL_controller = None
    else:
        DXL_controller = DynamixelController(robot.palm.motor_id, robot.DXL_port, robot.DXL_baudrate)

    if not EXTERNAL_ENC:
        # Force to bypass external encoders when EXTERNAL_ENC=None
        bypass_ext_enc = True

    # When debug, you might want to bypass external encoders
    if bypass_ext_enc:
        ext_enc = None
    else:
        ext_enc = MPS_Encoder_Cluster("MA310", BUS, robot.encoders, MAX_SPI_SPEED, SPI_MODE)

    motor_controller = MotorController(robot.BEAR_baudrate, robot.BEAR_port)

    # Ping all actuators at the beginning
    error = 0b0000  # 4 bit respectively for INDEX, INDEX_M, THUMB, Dynamixel

    # Ping all motors
    # Specify PING_TRAIL_COUNT in Constants_DAnTE
    # BEAR:
    for idx, f in enumerate(robot.fingerlist):
        trail = 0
        check = False
        print("Pinging %s..." % f.name)
        while not check:
            if not bool(motor_controller.pbm.ping(f.motor_id)[0]):
                trail += 1
                if trail > int(PING_TRAIL_COUNT / 2):
                    # WARNING about bad communication
                    print("WARNING: %s BEAR communication intermittent." % f.name)
                elif trail > PING_TRAIL_COUNT:
                    # Tried PING_TRAIL_COUNT times and still no luck:
                    print("ERROR: %s offline." % f.name)
                    error = error | (1 << idx)
                    break
                else:
                    # Retry in 0.5s
                    print("Retry in 0.5 second.")
                    time.sleep(0.5)
            else:
                # Ping succeed
                check = True
    # DXL:
    if not bypass_DXL:
        trail = 0
        check = False
        print("Pinging Palm actuator...")
        while not check:
            if not DXL_controller.ping():
                trail += 1
                if trail > int(PING_TRAIL_COUNT / 2):
                    # WARNING about bad communication
                    print("WARNING: Palm actuator communication intermittent.")
                if trail > PING_TRAIL_COUNT:
                    # Tried PING_TRAIL_COUNT times and still no luck:
                    print("ERROR: Palm actuator offline.")
                    error = error | (1 << 3)
                    break
                else:
                    # Retry in 0.5s
                    print("Retry in 0.5 second.")
                    time.sleep(0.5)
            else:
                # Ping succeed
                check = True

    if error:
        print("One or more actuators offline. Exiting...")
        return

    if not bypass_DXL:
        # Calibrate Palm actuator first
        # Home INDEX finger
        usr = input("Move all the finger to their home positions\nand put INDEX fingers in parallel, then press enter.")
        DXL_controller.set_homing_offset(0)
        robot.palm.home = DXL_controller.get_present_position()
        print("Palm home set.")
    else:
        robot.palm.home = 0

    motor_controller.init_driver_all()

    # Set Mode
    motor_controller.set_mode_all('velocity')

    # Set Limits and PID Gains
    for f in robot.fingerlist:
        # Limits
        motor_controller.pbm.set_limit_iq_max((f.motor_id, 1.5))
        motor_controller.pbm.set_limit_velocity_max((f.motor_id, 2*VEL_CAL))

        # Velocity PID
        motor_controller.pbm.set_p_gain_velocity((f.motor_id, VEL_P))
        motor_controller.pbm.set_i_gain_velocity((f.motor_id, VEL_I))
        motor_controller.pbm.set_d_gain_velocity((f.motor_id, VEL_D))
    # Position
    # motor_controller.pbm.set_p_gain_position((m_id, 0.15))

    # Home Dynamixel to the parallel position
    if bypass_DXL:
        usr = input("Move the index fingers to the parallel gesture then press enter.")
    else:
        # DXL not bypassed, automatic go to parallel.
        usr = input("Press enter to change to Pinch...")
        DXL_controller.set_mode("position")  # Position mode
        DXL_controller.torque_enable(1)  # Enable
        DXL_controller.set_goal_position(robot.palm.home+math.pi*7/15)
        time.sleep(0.5)
        usr = input("If everything looks alright press enter to continue, INDEX fingers will return to Parallel.\n"
                    "Otherwise, press E and enter to exit.")
        if usr == 'E' or usr == 'e':
            return
        else:
            DXL_controller.set_goal_position(robot.palm.home)
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
        motor_controller.torque_enable_all(1)
        time.sleep(0.1)
        # Goal Velocity
        for f in robot.fingerlist:
            if f.mirrored:
                goal_velocity = -VEL_CAL
            else:
                goal_velocity = VEL_CAL
            motor_controller.pbm.set_goal_velocity((f.motor_id, goal_velocity))

        home_i = [0, 0, 0]
        detect_count = [0, 0, 0]
        print("Looking for home...")
        while sum(running):
            try:
                # TODO: Replace with bulk_read
                status = motor_controller.pbm.get_bulk_status((INDEX.motor_id, 'present_velocity', 'present_iq'),
                                                              (INDEX_M.motor_id, 'present_velocity', 'present_iq'),
                                                              (THUMB.motor_id, 'present_velocity', 'present_iq'))
                err = [i[1] for i in status]
                vels = [i[0][0] for i in status]
                current = [i[0][1] for i in status]

                for i in range(3):
                    if abs(vels[i]) < VEL_CAL_DETECT and abs(current[i]) > IQ_CAL_DETECT:
                        if detect_count[i] > 5:
                            print("%s HOME reached." % robot.fingerlist[i].name)
                            # home_i[i] = current[i]
                            # print("%s home_i acquired." % robot.fingerlist[i].name)
                            motor_controller.torque_enable(robot.finger_ids[i], 0)
                            # print("End current was %2.2f" % home_i[i])
                            # print("End velocity was %2.2f" % vels[i])
                            running[i] = False
                        else:
                            detect_count[i] += 1
                    if err[i] != 128:
                        print("%s BEAR error, code:" % robot.fingerlist[i].name, bin(err[i]))

            except KeyboardInterrupt:
                running = [0]
                motor_controller.torque_enable_all(0)
                print("User interrupted.")

    # Check if HOMING_OFFSET need to be updated
    finger_homing_error = []
    for idx, f in enumerate(robot.fingerlist):
        present_pos = motor_controller.pbm.get_present_position(f.motor_id)[0][0][0]
        # pdb.set_trace()
        if abs(present_pos) > 0.15:
            # If the present position is away from zero for more than 0.05rad (~2.8deg)
            finger_homing_error.append([f, present_pos])
    if finger_homing_error:
        # If there is any finger that needs update
        print("The following finger's HOMING_OFFSET needs to be updated:")
        for i in finger_homing_error:
            print(i[0].name+", homing error: "+str(i[1]))
        usr = input("Press ENTER to continue, or I if you insist to ignore.")
        if usr == 'I':
            finger_homing_error = []
    else:
        # No finger needs update
        print("All HOMING_OFFSET settings are fine.")
        usr = input("Press ENTER to continue, or C if you insist to reset HOMING_OFFSET for all fingers.")
        if usr == 'C':
            # User insist to reset HOMING_OFFSET for all fingers
            for f in robot.fingerlist:
                finger_homing_error.append([f, 0])

    if finger_homing_error:
        # If there is any finger to be updated
        # Clear HOMING_OFFSET
        for i in finger_homing_error:
            m_id = i[0].motor_id
            motor_controller.pbm.set_homing_offset((m_id, 0))
            # Check setting
            check = False
            trial_count = 1
            # debug_temp = motor_controller.pbm.get_homing_offset(m_id, m_id, m_id)
            while not check:
                try:
                    if abs(motor_controller.pbm.get_homing_offset(m_id)[0][0][0]) < 1:
                        check = True
                        print("HOMING_OFFSET cleared for %s. Trails: %d." % (i[0].name, trial_count))
                    else:
                        motor_controller.pbm.set_homing_offset((m_id, 0))
                        time.sleep(0.05)
                        trial_count += 1
                except KeyboardInterrupt:
                    check = True
                    print("User interrupted.")
        # Wait for 0.2 sec after setting HOMING_OFFSET
        time.sleep(0.2)

        # Get home_offset
        for i in finger_homing_error:
            i[0].homing_offset = -(motor_controller.pbm.get_present_position(i[0].motor_id)[0][0][0])
        # print(home_offset)

        # Set Homing_Offset
        for i in finger_homing_error:
            m_id = i[0].motor_id
            homing_offset = i[0].homing_offset
            motor_controller.pbm.set_homing_offset((m_id, homing_offset))
            # time.sleep(0.05)
            # Check setting
            check = False
            trial_count = 1
            while not check:
                try:
                    temp = motor_controller.pbm.get_homing_offset(m_id)[0][0][0]
                    print("Current homing_offset: % 2.2f" % temp)
                    if abs(motor_controller.pbm.get_homing_offset(m_id)[0][0][0] - homing_offset) < 0.01:
                        check = True
                        print("HOMING_OFFSET updated for %s. Trails: %d." % (i[0].name, trial_count))
                    else:
                        motor_controller.pbm.set_homing_offset((m_id, homing_offset))
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
            pos = motor_controller.pbm.get_present_position(m_id)[0][0][0]
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
    for f in robot.fingerlist:
        f.homing_offset = motor_controller.pbm.get_homing_offset(f.motor_id)[0][0][0]

    if not bypass_ext_enc:
        ext_enc.connect()
        time.sleep(0.5)
        data = ext_enc.get_angle()
        ext_enc.release()
        for idx, f in enumerate(robot.fingerlist):
            f.encoder_offset = data[idx]

    # Get end_pos
    usr = input("Auto set the end limit? (y/n)")
    if usr == "n" or usr == "N":
        usr = input("Move all the fingers to the end limit and press enter.\n")
        end_pos = motor_controller.pbm.get_present_position(INDEX.motor_id, INDEX_M.motor_id, THUMB.motor_id)
        for idx, i in enumerate(end_pos):
            robot.fingerlist[idx].travel = i[0][0]
            print("end_pos recorded as:", robot.fingerlist[idx].name, robot.fingerlist[idx].travel)

    elif usr == "y" or usr == "Y":
        # Look for End-Pos automatically
        running = [True, True, True]
        # Enable Torque
        motor_controller.torque_enable_all(1)
        time.sleep(0.1)
        # Goal Velocity
        for f in robot.fingerlist:
            if f.mirrored:
                goal_velocity = VEL_CAL
            else:
                goal_velocity = -VEL_CAL
            motor_controller.pbm.set_goal_velocity((f.motor_id, goal_velocity))

        time.sleep(0.5)
        end_i = [0, 0, 0]
        detect_count = [0, 0, 0]
        print("Looking for end_pos...")
        while sum(running):
            try:
                # TODO: Replace with bulk_read
                status = motor_controller.pbm.get_bulk_status((INDEX.motor_id, 'present_velocity', 'present_iq'),
                                                              (INDEX_M.motor_id, 'present_velocity', 'present_iq'),
                                                              (THUMB.motor_id, 'present_velocity', 'present_iq'))
                err = [i[1] for i in status]
                vels = [i[0][0] for i in status]
                current = [i[0][1] for i in status]

                for i in range(3):
                    if abs(vels[i]) < VEL_CAL_DETECT and abs(current[i]) > IQ_CAL_DETECT:
                        if detect_count[i] > 20:
                            # TODO: check total travel value
                            print("%s end_pos reached." % robot.fingerlist[i].name)
                            robot.fingerlist[i].travel = \
                                motor_controller.pbm.get_present_position(robot.finger_ids[i])[0][0][0]
                            print("%s end_pos acquired." % robot.fingerlist[i].name)
                            end_i[i] = current[i]
                            print("%s end_i acquired." % robot.fingerlist[i].name)
                            motor_controller.torque_enable(robot.finger_ids[i], 0)
                            print("%s end_pos recorded as: % 8.2f" % (robot.fingerlist[i].name, robot.fingerlist[i].travel))
                            print("End current was %2.2f" % end_i[i])
                            print("End velocity was %2.2f" % vels[i])
                            running[i] = False
                        else:
                            detect_count[i] += 1
                    if err[i] != 128:
                        print("%s BEAR error, code:" % robot.fingerlist[i].name, bin(err[i]))

            except KeyboardInterrupt:
                running = [0]
                motor_controller.torque_enable_all(0)
                end_pos = [0, 0, 0]
                print("User interrupted.")

    usr = input("Reset fingers? (y/n)")
    if usr == "y" or usr == "Y":
        for f in robot.fingerlist:
            motor_controller.pbm.set_p_gain_position((f.motor_id, POS_P))
            motor_controller.pbm.set_i_gain_position((f.motor_id, POS_I))
            motor_controller.pbm.set_d_gain_position((f.motor_id, POS_D))
            motor_controller.pbm.set_limit_iq_max((f.motor_id, 1))
        time.sleep(0.2)
        print("Fingers resetting...")

        # Set Mode and Limit
        motor_controller.set_mode_all('position')

        running = [True, True, True]
        # Enable torque and go to Home
        motor_controller.torque_enable_all(1)
        motor_controller.pbm.set_goal_position((THUMB.motor_id, 0.001),
                                               (INDEX.motor_id, 0.001),
                                               (INDEX_M.motor_id, 0.001))
        time.sleep(1)
        while sum(running):
            try:
                status = motor_controller.pbm.get_bulk_status((INDEX.motor_id, 'present_position', 'present_velocity'),
                                                              (INDEX_M.motor_id, 'present_position', 'present_velocity'),
                                                              (THUMB.motor_id, 'present_position', 'present_velocity'))
                err = [data[1] for data in status]
                position = [data[0][0] for data in status]
                velocity = [data[0][1] for data in status]
                for i in range(3):
                    if abs(position[i]) < 0.05:
                        running[i] = False
                        motor_controller.torque_enable(robot.finger_ids[i], 0)
                    if err[i] != 128:
                        print("%s BEAR error, code:" % robot.fingerlist[i].name, bin(err[i]))
            except KeyboardInterrupt:
                running = [0]
                print("User interrupted.")

    motor_controller.torque_enable_all(0)
    if not bypass_DXL:
        DXL_controller.torque_enable(0)

    print("Calibration summary:")
    for f in robot.fingerlist:
        print("%s Motor ID: %d, homing_offset: % 8.2f, travel: % 8.2f, encoder_offset: % 8.2f"
              % (f.name, f.motor_id, f.homing_offset, f.travel, f.encoder_offset))
    if finger_homing_error:
        print("The following fingers' homing_offset has been updated:")
        print([i[0].name for i in finger_homing_error])
        usr = input("It is recommended to save changed settings. Continue? (y/n)")
        if usr == 'Y' or 'y':
            for i in finger_homing_error:
                motor_controller.pbm.save_config(i[0].motor_id)
            print("Config saved to BEAR.")
        else:
            print("Abort saving to BEAR. Please power cycle hardware before using.")

    usr = input("Save Data? (y/n)")
    if usr == "y" or usr == "Y":
        # Build list
        data = []
        for f in robot.fingerlist:
            data.append([f.name, f.motor_id, f.homing_offset, f.travel, f.encoder_offset])
        data.append([robot.palm.name, robot.palm.motor_id, robot.palm.home, robot.palm.travel])
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

