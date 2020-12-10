# DAnTE_V2
Controller software for Westwood Robotics DAnTE V2

## Dependencies

#### 1. PyBEAR

Please use PyBEAR from Westwood Robotics [PyBEAR-WR](https://github.com/Westwood-Robotics/PyBEAR-WR), and make sure that you are on the develop branch. 

Due to the differences in data return format, the current RoMeLa PyBEAR will **NOT** work with DAnTE V2.

#### 2. Dynamixel SDK

The Dynamixel SDK is also required to communicate with the Robotis X series actuator that is driving the index fingers on DAnTE V2. You can download it here:[Dynamixel SDK](https://github.com/ROBOTIS-GIT/DynamixelSDK)

#### 3. Udev Rules

Using the Boosted USB2BEAR/USB2RoMeLa device to communicate with BEAR, must add 00-WestwoodRobotics.rules from PyBEAR-WR_Rev first.
Use a generic FT232 dangle to communicate with Dynamixel in palm, add Util/00-WR_DAnTE.rules before using
Modify 00-WR_DAnTE.rules accordingly, if needed.

Move 00-WR_DAnTE.rules file into /etc/udev/rules.d/ with 'sudo cp'
Reload the rules
```bash
sudo udevadm control --reload
```

## Work with DAnTE

#### 0. Preperation

###### 0.0 Robot.py

###### 0.1 Calibration

Perform calibration before your first run after open-box, and then after every repair/maintemance, or as prompt during usage. To calibrate, cd into ./DAnTE_V2/ directory and run calibration.py(assuming python3):
```bash
python3 -m Play.calibration
```

Then follow instructions as prompt. You can select to calibrate the whole hand or just one single finger.

The calibration checks the integraty of the system and sets zero references for all actuators. Then it looks for the range of motion for all the fingers or the selected finger. Calibration results will be writen into ../Settings/initials.txt for initialization reference.

###### 0.2 Initialization

Initialization is a function defined in the robot_controller module. Run initialization first after booting the system. Make sure system is started with start_robot() function.

The initialization checks if there is any discrepancy between the settings in the actuators registor and the data of the robot instance, then it quickly run all fingers through their rang of motion to make sure nothing is stuck. System will not function without passing initialization.

Initialization example:
```
from Settings.Robot import *
from Play.robot_controller import RobotController

rc = RobotController(robot=DAnTE)
rc.start_robot()
rc.initialization()
```

#### 1. Motions

To run DAnTE V2, first create an instance of RobotController:
```
from Settings.Robot import *
from Play.robot_controller import RobotController

rc = RobotController(robot=DAnTE)
```
The default robot for RobotController is DAnTE, thus you can also omit that setting, and simply do:
```
rc = RobotController()
```

###### 1.0 Start the system

Before you can do anything with DAnTE, you must start the system with start_robot() function:
```
rc.start_robot()
```
This function pings all actuators, then reads the initals.txt and populate the variables of the robot instance.

###### 1.1 Grab

Use grab(gesture, mode, \**options) function for object grabbing. 

This function controls the grab motion of DAnTE:
- Grab with a gesture, tripod(Y), pinch(I) or parallel(P)
- Specify a grab mode: (H)old or (G)rip
  - In Hold mode, DAnTE will just hold the object with a big damping but nearly no gripping force. The object can escape relatively easily. Damping is adjected according to the final_strength setting.
  - In Grip mode, the fingers will move further 'into' the object to create a firm grip, until the final_strength is reached. In this mode, the final_strength specify the final gripping current(A) on each finger. In Parallel gesture, the final gripping current(A) on the THUMB is doubled for force balance purpose.
- Optional kwargs: (if not specified, go with a default value)
  - Approach speed (approach_speed), default = 1
  - Approach stiffness (approach_stiffness), default = 1
  - Detection current (detect_current), default = 0.42
  - Grip force/Hold stiffness (final_strength), default = 1
  - Maximum torque current (max_iq), default = 1.5
  - Data log function (logging), default = False

###### 1.2 Release

Use release() function for releasing.
**This function is currently under construction.**

#### 2. Tunning

Tune DAnTE via Macros_DAnTE, rarely need to modify Contants_DAnTE.


## Future work

- Interactive operation synced with oeprator with force

- Raspberry Zero integration, and add a higher level

- Full hand kinematics and object size/shape estimation after grabbing.

- Reboot and shutdown functions

