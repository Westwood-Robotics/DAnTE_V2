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
from pathlib import Path
from Play.motor_controller import MotorController
from Play.dynamixel_controller import DynamixelController
from Settings.Robot import *
from Play.MPS import MPS_Encoder_Cluster
from Settings.Constants_DAnTE import *
from Play.MPS import MPS_Encoder
import pdb

# -----------------------------
# Function for finger calibration


def calibration_single(f, motor_controller):
    # Single finger calibration.
    # Specify the finger with f
    # TODO: if encoder is not None, read encoder initials

    m_id = f.motor_id
    motor_controller.init_driver(m_id)

    # Set Mode
    motor_controller.set_mode(m_id, 'velocity')

    # Set Limits
    motor_controller.pbm.set_limit_iq_max((m_id, 1.5))
    motor_controller.pbm.set_limit_velocity_max((m_id, 2*VEL_CAL))

    # Set PID Gains
    # Velocity
    motor_controller.pbm.set_p_gain_velocity((m_id, 0.2))
    motor_controller.pbm.set_i_gain_velocity((m_id, 0.0005))

    # Position
    motor_controller.pbm.set_p_gain_position((m_id, 0.15))

    # Home finger
    usr = input("Move the finger to the home position press enter.\n")

    # Clear HOMING_OFFSET
    motor_controller.pbm.set_homing_offset((m_id, 0))
    # Check setting
    check = False
    trial_count = 1
    while not check:
        try:
            if abs(motor_controller.pbm.get_homing_offset(m_id)[0][0]) < 1:
                check = True
                print("HOMING_OFFSET cleared. Trails: %d." % trial_count)
            else:
                motor_controller.pbm.set_homing_offset((m_id, 0))
                time.sleep(0.05)
                trial_count += 1
        except KeyboardInterrupt:
            check = True
            print("User interrupted.")
    # Wait for 1 sec after setting HOMING_OFFSET
    time.sleep(1)

    # Get home_offset
    home_offset = -motor_controller.pbm.get_present_position(m_id)[0][0]
    # print(home_offset)

    # Set Homing_Offset
    motor_controller.pbm.set_homing_offset((m_id, home_offset))
    # time.sleep(0.05)
    # Check setting
    check = False
    trial_count = 1
    while not check:
        try:
            temp = motor_controller.pbm.get_homing_offset(m_id)[0][0]
            print("home_offset set as % 8.2f" % temp)
            if abs(motor_controller.pbm.get_homing_offset(m_id)[0][0] - home_offset) < 1:
                check = True
                print("HOMING_OFFSET updated. Trails: %d." % trial_count)
            else:
                motor_controller.pbm.set_homing_offset((m_id, home_offset))
                # time.sleep(0.05)
                trial_count += 1
        except KeyboardInterrupt:
            check = True
            print("User interrupted.")
    # Wait for 1 sec after setting HOMING_OFFSET
    time.sleep(1)

    # Final check
    pos = motor_controller.pbm.get_present_position(m_id)[0][0]
    if abs(pos) < 0.15:
        print("Finger homed.")
    else:
        print("Homing abnormal!")  # TODO: throw exception

    print("home_offset recorded as: % 8.2f" % home_offset)
    print("Current Position:%5.2f" % pos)

    # Get end_pos
    user = input("Do you want to set the end limit manually? (y/n)")
    if user == "y" or user == "Y":
        user = input("Move the finger to the end limit and press enter.\n")
        end_pos = motor_controller.pbm.get_present_position(m_id)[0][0]
        print("end_pos recorded as: % 8.2f" % end_pos)
    elif user == "n" or user == "N":
        # Look for End-Pos automatically
        run = True
        # Enable Torque
        motor_controller.torque_enable(m_id, 1)
        time.sleep(0.1)
        # Goal Velocity
        if f.mirrored:
            goal_velocity = 5
        else:
            goal_velocity = -5  # -5 for THUMB and INDEX
        motor_controller.pbm.set_goal_velocity((m_id, goal_velocity))
        time.sleep(0.5)
        print("Looking for end_pos...")
        while run:
            try:
                status = motor_controller.pbm.get_bulk_status((m_id, 'present_velocity', 'present_iq'))
                vel = status[0][0][0]
                current = status[0][0][1]
                if abs(vel)<0.1 and abs(current) > 1:
                    print("end_pos reached.")
                    run = False
                    end_pos = motor_controller.pbm.get_present_position(m_id)[0][0]
                    print("end_pos acquired.")
                    end_i = motor_controller.pbm.get_present_iq(m_id)[0][0]
                    print("end_i acquired.")
                    motor_controller.torque_enable(m_id, 0)
                    print("end_pos recorded as: % 8.2f" % end_pos)
                    print("End current was %2.2f" % end_i)

                # vel = motor_controller.pbm.get_present_velocity(m_id)[0][0]
                # print("Vel acquired. %2.2f" % abs(vel))
                # if abs(vel) < 0.1:
                #     print("Vel is small.")
                #     time.sleep(0.5)
                #     current = motor_controller.pbm.get_present_iq(m_id)[0][0]
                #     print("Current acquired.")
                #     if abs(current) > 1:
                #         print("Current is high.")
                #         run = False
                #         end_pos = motor_controller.pbm.get_present_position(m_id)[0][0]
                #         print("end_pos acquired.")
                #         end_i = motor_controller.pbm.get_present_iq(m_id)[0][0]
                #         print("end_i acquired.")
                #         motor_controller.torque_enable(m_id, 0)
                #         print("end_pos recorded as: % 8.2f" % end_pos)
                #         print("End current was %2.2f" % end_i)
            except KeyboardInterrupt:
                run = False
                motor_controller.torque_enable(m_id, 0)
                end_pos = None
                print("User interrupted.")

    # # Set position limits
    # if end_pos is None:
    #  print("User interrupted.")
    # else:
    #  limit_margin = 450
    #  # Set Finger Limit
    #  if goal_velocity<0:
    #      motor_controller.pbm.set_limit_position_min((m_id,end_pos-limit_margin))
    #      bear.set_limit_position_max((m_id,limit_margin))
    #  else:
    #      bear.set_limit_position_min((m_id,-limit_margin))
    #      bear.set_limit_position_max((m_id,end_pos+limit_margin))

    # Reset
    # Set Position Gain
    motor_controller.pbm.set_p_gain_position((m_id, 0.15))
    motor_controller.pbm.set_i_gain_position((m_id, 0))
    motor_controller.pbm.set_d_gain_position((m_id, 0.05))

    user = input("Reset finger? (y/n)")
    if user == "y" or user == "Y":
        time.sleep(0.2)
        print("Finger resetting...")
        # Set Mode and Limit
        motor_controller.set_mode(m_id, 'position')
        motor_controller.pbm.set_limit_iq_max((m_id, 1.8))
        run = True
        # Enable torque and go to Home
        motor_controller.torque_enable(m_id, 1)
        motor_controller.pbm.set_goal_position((m_id, 150))
        time.sleep(0.5)
        # pdb.set_trace()
        while run:
            try:
                vel = motor_controller.pbm.get_present_velocity(m_id)[0][0]
                if abs(vel) < 0.1:
                    run = False
                    time.sleep(0.5)
                    motor_controller.torque_enable(m_id, 0)
                #time.sleep(0.05)
            except KeyboardInterrupt:
                run = False
                print("User interrupted.")

    # Build list
    positions = str("%d,%f,%f" % (m_id, home_offset, end_pos))
    # Write file
    filename = 'Settings/initials_single.txt'
    filepath = os.path.join(str(Path(os.getcwd()).parent), filename)
    initials = open(filepath, 'w')
    initials.write(positions)
    initials.close()
    print("The following data has been written into initials_single.txt:")
    print("Motor ID: %d" % m_id, "home_offset: % 8.2f" % home_offset, "end_pos: % 8.2f" % end_pos)
    print("Finger calibration complete.")


def calibration_full(robot, bypass_DXL=False, bypass_ext_enc=False):
    # Calibrate the whole hand.

    # When debug, you might want to bypass Dynamixel
    if bypass_DXL:
        DXL_controller = None
    else:
        DXL_controller = DynamixelController(robot.palm.motor_id, robot.DXL_port, robot.DXL_baudrate)

    # When debug, you might want to bypass external encoders
    if bypass_ext_enc:
        ext_enc = None
    else:
        ext_enc = MPS_Encoder_Cluster("MA310", BUS, robot.encoders, MAX_SPI_SPEED, SPI_MODE)

    motor_controller = MotorController(robot.BEAR_baudrate, robot.BEAR_port)

    # Ping all actuators at the beginning
    error = 0b0000  # 4 bit respectively for INDEX, INDEX_M, THUMB, Dynamixel

    # Ping all motors
    for idx, f in enumerate(robot.fingerlist):
        if not bool(motor_controller.pbm.ping(f.motor_id)):
            print("%s offline." % f.name)
            error = error | (1 << idx)
    if not bypass_DXL:
        if not DXL_controller.ping():
            print("Palm actuator offline.")
            error = error | (1 << 3)

    if error:
        print("One or more actuators offline. Exiting...")
        return

    # Calibrate Palm actuator first
    # Home INDEX finger
    usr = input("Move all the finger to their home positions\nand put INDEX fingers in parallel, then press enter.")
    DXL_controller.set_homing_offset(0)
    robot.palm.home = DXL_controller.get_present_position()
    print("Palm home set.")

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
                            home_i[i] = current[i]
                            print("%s home_i acquired." % robot.fingerlist[i].name)
                            motor_controller.torque_enable(robot.finger_ids[i], 0)
                            print("End current was %2.2f" % home_i[i])
                            print("End velocity was %2.2f" % vels[i])
                            running[i] = False
                        else:
                            detect_count[i] += 1
                    if err[i] != 128:
                        print("%s error, code:" % robot.fingerlist[i].name, bin(err[i]))

            except KeyboardInterrupt:
                running = [0]
                motor_controller.torque_enable_all(0)
                print("User interrupted.")

    # Clear HOMING_OFFSET
    for f in robot.fingerlist:
        m_id = f.motor_id
        motor_controller.pbm.set_homing_offset((m_id, 0))
        # Check setting
        check = False
        trial_count = 1
        # debug_temp = motor_controller.pbm.get_homing_offset(m_id, m_id, m_id)
        while not check:
            try:
                if abs(motor_controller.pbm.get_homing_offset(m_id)[0][0]) < 1:
                    check = True
                    print("HOMING_OFFSET cleared for %s. Trails: %d." % (f.name, trial_count))
                else:
                    motor_controller.pbm.set_homing_offset((m_id, 0))
                    time.sleep(0.05)
                    trial_count += 1
            except KeyboardInterrupt:
                check = True
                print("User interrupted.")
    # Wait for 0.2 sec after setting HOMING_OFFSET
    time.sleep(0.2)
    print("HOMING_OFFSET all cleared.")

    # Get home_offset
    # TODO: replace with bulk_read
    for f in robot.fingerlist:
        f.homing_offset = -(motor_controller.pbm.get_present_position(f.motor_id)[0][0])
    # print(home_offset)

    # Set Homing_Offset
    for f in robot.fingerlist:
        m_id = f.motor_id
        homing_offset = f.homing_offset
        motor_controller.pbm.set_homing_offset((m_id, homing_offset))
        # time.sleep(0.05)
        # Check setting
        check = False
        trial_count = 1
        while not check:
            try:
                temp = motor_controller.pbm.get_homing_offset(m_id)[0][0]
                print("Current homing_offset: % 2.2f" % temp)
                if abs(motor_controller.pbm.get_homing_offset(m_id)[0][0] - homing_offset) < 0.01:
                    check = True
                    print("HOMING_OFFSET updated for %s. Trails: %d." % (f.name, trial_count))
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
    for f in robot.fingerlist:
        m_id = f.motor_id
        pos = motor_controller.pbm.get_present_position(m_id)[0][0]
        if abs(pos) < 0.01:
            print("%s homed." % f.name)
        else:
            print("%s homing abnormal!" % f.name)  # TODO: throw exception
            usr = input("Continue? (Y/N)")
            if usr == 'Y' or 'y':
                pass
            else:
                return

    # print("home_offset recorded as: % 8.2f" % home_offset)
    # print("Current Position:%5.2f" % pos)

    if not bypass_ext_enc:
        ext_enc.connect()
        time.sleep(0.5)
        data = ext_enc.read_angle()
        ext_enc.release()
        for idx, f in enumerate(robot.fingerlist):
            f.encoder_offset = data[idx]

    # Get end_pos
    usr = input("Auto set the end limit? (y/n)")
    if usr == "n" or usr == "N":
        usr = input("Move all the fingers to the end limit and press enter.\n")
        end_pos = motor_controller.pbm.get_present_position(INDEX.motor_id, INDEX_M.motor_id, THUMB.motor_id)
        for idx, i in enumerate(end_pos):
            robot.fingerlist[idx].travel = i[0]
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
                            robot.fingerlist[i].travel = motor_controller.pbm.get_present_position(robot.finger_ids[i])[0][0]
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
                        print("%s error, code:" % robot.fingerlist[i].name, bin(err[i]))

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
                        print("%s error, code:" % robot.fingerlist[i].name, bin(err[i]))
            except KeyboardInterrupt:
                running = [0]
                print("User interrupted.")

    motor_controller.torque_enable_all(0)
    DXL_controller.torque_enable(0)

    # Build list
    data = []
    for f in robot.fingerlist:
        data.append([f.name, f.motor_id, f.homing_offset, f.travel, f.encoder_offset])
    data.append([robot.palm.name, robot.palm.motor_id, robot.palm.home, robot.palm.travel])

    usr = input("Save Data? (y/n)")
    if usr == "y" or usr == "Y":
        # Write file
        filename = 'Settings/initials.txt'
        filepath = os.path.join(str(Path(os.getcwd())), filename)
        initials = open(filepath, 'w')
        for i in data:
            initials.write(str(i)[1:-1])
            initials.write('\n')
        initials.close()
        print("The following data has been written into initals.txt:")
        for f in robot.fingerlist:
            print("%s Motor ID: %d, homing_offset: % 8.2f, travel: % 8.2f, encoder_offset: % 8.2f"
                  % (f.name, f.motor_id, f.homing_offset, f.travel, f.encoder_offset))
            motor_controller.pbm.save_config(f.motor_id)
        print("Config saved to BEAR.")
        print("Full hand calibration complete.")
    else:
        print("Abort saving.")


if __name__ == '__main__':
    Robot = DAnTE

    user = input("Is this a (F)ull hand calibration or a (S)ingle finger calibration?")
    if user == 'F' or user == 'f':
        print("Starting full hand calibration...")
        calibration_full(Robot)

    elif user == 'S' or user == 's':
        BEAR_controller = MotorController(Robot.BEAR_baudrate, Robot.BEAR_port)
        print("Starting single finger calibration...")
        user = input("Which finger to calibrate? [(T)HUMB, (I)NDEX, INDEX_(M)]\n")
        if user == 'T' or user == 't':
            Finger = THUMB
        elif user == 'I' or user == 'i':
            Finger = INDEX
        elif user == 'M' or user == 'm':
            Finger = INDEX_M
        else:
            print("Invalid input. Exit.")
            exit()
        print("Calibrating "+Finger.name+"...")
        calibration_single(Finger, BEAR_controller)
