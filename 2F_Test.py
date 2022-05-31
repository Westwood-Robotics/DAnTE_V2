from Play.robot_controller import RobotController
from Settings.Robot import *
import time

run = False
rc = RobotController(robot=DAnTE_2F)
rc.start_robot()
rc.calibration_geometry()
rc.initialization()
rc.change_gesture('Y')

usr = input("Hit enter to grab...")
rc.grab('G', approach_speed=2, preload=0.1, final_stiffness=0.2)
usr = input("Hit enter to release...")
rc.release('F')
usr = input("Hit enter to grab...")
rc.grab('H', approach_speed=4, preload=0.5, final_stiffness=5)
usr = input("Hit enter to release...")
rc.release('L')
usr = input("Hit enter to release...")
rc.release('F')