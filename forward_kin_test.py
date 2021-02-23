from Settings.Robot import *
from Play.robot_controller import RobotController

rc = RobotController(robot=DAnTE)
rc.start_robot()
rc.initialization()
usr = input("Press any key and enter to continue...")
rc.update_angles()
