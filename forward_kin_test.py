from Settings.Robot import *
from Play.robot_controller import RobotController

rc = RobotController(robot=DAnTE)
rc.start_robot()
rc.initialization()

rc.update_angles()