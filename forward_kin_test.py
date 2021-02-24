from Settings.Robot import *
from Play.robot_controller import RobotController

rc = RobotController(robot=DAnTE)
rc.start_robot()
rc.initialization()
run = True
while run:
    try:
        usr = input("Press any key and enter to continue...")
        rc.update_angles()
        rc.visualization()
    except KeyboardInterrupt:
        print("User interrupted.")
        run = False
