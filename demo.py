#!usr/bin/env python3
__author__ = "Xiaoguang Zhang"
__email__ = "xzhang@westwoodrobotics.net"
__copyright__ = "Copyright 2020 Westwood Robotics"
__date__ = "Jul 8, 2021"
__version__ = "0.1.0"
__status__ = "Beta"

# Run this script to perform a system demo. Choose options as prompted.

from Play.robot_controller import RobotController
from Settings.Robot import *
import time

run = False
rc = RobotController()
rc.start_robot()
if rc.initialization():
    run = True
else:
    print("Initialization failed, please debug. Exiting...")
    exit()

while run:
    print("(a) Business Card\n"
          "(b) Potato Chip\n"
          "(c) Soft Tripod\n"
          "(d) Soft Pinch\n"
          "(e) Power Grip\n"
          "(f) Power Hold\n"
          "(z) Customize...\n"
          "(S) STOP\n")
    usr = input("Select from the above demo options:")

    if usr == 'a':
        print("Business Card demo, starting in 1 second ...")
        time.sleep(1)
        rc.grab('I', 'G', approach_speed=3, preload=0.1, final_stiffness=0.2)
    elif usr == 'b':
        print("Potato Chip demo, starting in 1 seconds ...")
        time.sleep(1)
        rc.grab('Y', 'H', approach_speed=2.5, preload=0.1, final_stiffness=0.2)
    elif usr == 'c':
        print("Soft Tripod demo, starting in 1 seconds ...")
        time.sleep(1)
        rc.grab('Y', 'G', approach_speed=3, preload=0.1, final_stiffness=0.25)
    elif usr == 'd':
        print("Soft Pinch demo, starting in 1 seconds ...")
        time.sleep(1)
        rc.grab('I', 'G', approach_speed=2.5, preload=0.1, final_stiffness=0.2)
    elif usr == 'e':
        print("Power Grip demo, starting in 1 seconds ...")
        time.sleep(1)
        rc.grab('P', 'G', approach_speed=4, preload=1, final_stiffness=5)
    elif usr == 'f':
        print("Power Hold demo, starting in 1 seconds ...")
        time.sleep(1)
        rc.grab('P', 'H', approach_speed=4, preload=0.5, final_stiffness=5)

    elif usr == 'z':
        print("Customized parameters...")
        gesture = None
        while not gesture:
            gesture = input("Select gesture (P)arallel, (Y)Tripod, (I)Pinch :")
            if gesture not in ['P', 'Y', 'I']:
                print("Invalid selection, please try again.")
                gesture = None
        mode = None
        while not mode:
            mode = input("Select mode (H)old, (G)rip :")
            if mode not in ['H', 'G']:
                print("Invalid selection, please try again.")
                mode = None
        Approach_Speed = input("Input approach speed or press ENTER to use default:")
        if not Approach_Speed:
            Approach_Speed = -1
        Stiffness = input("Input final stiffness or press ENTER to use default:")
        if not Stiffness:
            Stiffness = -1
        Preload = input("Input preload or press ENTER to use default:")
        if not Preload:
            Preload = -1
        input("Press ENTER to start...")
        rc.grab(gesture, mode, approach_speed=Approach_Speed, preload=Preload, final_stiffness=Stiffness)

    elif usr == 'S':
        run = False
        continue
    else:
        print("Invalid selection, please try again.")
        continue

    usr = input("Press ENTER to let go.")
    rc.release('L')
    usr = input("Press ENTER to release.")
    delta_time

rc.release()
rc.set_robot_enable(0)
print("Demo stopped. Thanks for using DAnTE.")



