from Play.robot_controller import RobotController

rc = RobotController()
rc.start_robot()
rc.initialization()

input("Press Enter to grab Strawberry...")
rc.grab('Y', 'G', approach_speed=5, preload=0.1, final_stiffness=0.3)
input("Press Enter to let go...")
rc.release('L')
input("Press Enter to release...")
rc.release()

rc.change_gesture('I')

input("Press Enter to grab Cherry...")
rc.grab('I', 'G', approach_speed=2.5, preload=0.1, final_stiffness=0.2)
input("Press Enter to let go...")
rc.release('L')
input("Press Enter to release...")
rc.release()

rc.change_gesture('Y')

input("Press Enter to grab Tomato...")
rc.grab('Y', 'G', approach_speed=3, preload=0.3, final_stiffness=0.2)
input("Press Enter to let go...")
rc.release('L')
input("Press Enter to release...")
rc.release()

input("Press Enter to grab Little Net Ladle...")
rc.grab('Y', 'G', approach_speed=5, preload=0.5, final_stiffness=0.7)
input("Press Enter to let go...")
rc.release('L')
input("Press Enter to release...")
rc.release()

input("Press Enter to grab Big net ladle...")
rc.grab('Y', 'H', approach_speed=3, preload=0.5, final_stiffness=1)
input("Press Enter to let go...")
rc.release('L')
input("Press Enter to release...")
rc.release()
rc.set_robot_enable(0)

