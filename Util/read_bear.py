from Play.motor_controller import MotorController
from Settings.Robot import *
from Settings.Constants_DAnTE import *

robot = DAnTE # Define the robot

MC = MotorController(robot.BEAR_baudrate, robot.BEAR_port)

while True:
    try:
        status = MC.pbm.get_bulk_status((INDEX.motor_id, 'present_position'),
                                        (INDEX_M.motor_id, 'present_position'),
                                        (THUMB.motor_id, 'present_position'))
        err = [data[1] for data in status]
        position = [data[0][0] for data in status]

        # Print positions
        # print("INDEX: %2.2f, INDEX_M: %2.2f, THUMB: %2.2f" % position)
        print(position)
        time.sleep(0.5)
    except KeyboardInterrupt:
        running = [0]
        print("User interrupted.")