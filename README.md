# DAnTE_V2
Controller software for Westwood Robotics DAnTE V2

Firmware version 0.1.1

## Hardware
#### 1. DAnTE_V2

The current version of DAnTE V2 has three fingers: (start from the fixed finger and count clockwise when looking at the palm from the top) THUMB, INDEX, INDEX_M. All fingers has 3 joints, 2 DoF and are under actuated, driven by one Koala BEAR V2 actuator by Westwood Robotics. The INDEX and INDEX_M fingers are mirrored to each other, and are mirrorly coupled and driven by a single Dynamixel servo motor by Robotis so that they can respectively rotate along their axis perpendicular to the palm. The BEARs communicate with controlling device via Westwood Robotics USB2BEAR dangle, and an ordinary RS232 device is needed for the Dynamixel. 

Each finger has an external encoder (12bit Encoder-R12S10 by Westwood Robotics) at the base joint (MCP joint) of the finger. Absolute kinematics of a finger can be acquired when combine the external encoder reading and the BEAR position reading. The encoders use SPI protocal to communicate with controlling device.

DAnTE_V2 takes 9~12V DC power supply. Lower voltage will cause hardware fault while higher voltage will cause hardware damage upon power up.

#### 2. Controlling Device

This current DAnTE firmware is only written in Python 3 thus a computer with Python 3 environment is required. 

DAnTE is controlled by a Raspberry Pi 4 by default, but it can also be paired with other computers that has at least equivalent computing power. Keep in mind that an SPI interface is required to communicate with the external encoders, otherwise, please go to Settings/Constants_DAnTE.py and set "EXTERNAL_ENC" to "None".

## Dependencies

#### 1. PyBEAR

Please use PyBEAR from Westwood Robotics [PyBEAR-WR](https://github.com/Westwood-Robotics/PyBEAR-WR).

Must use PyBEAR 0.1.1 or higher.

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

#### 4. SciPy

The SciPy package is required for kinematics. Install with pip before playing with DAnTE. 

#### 5. WiringPi & spidev

The WiringPi and spidev packages are required for reading external encoders. 
WiringPi should be PRE-INSTALLED with standard Raspbian systems. Otherwise, refer to http://wiringpi.com/download-and-install/
spidev should also come with standard Raspbian system.

If you are not controlling DAnTE from a Raspberry Pi(thus not using external encoders to get absolute exact joint angles of the fingers), please go to Settings/Constants_DAnTE.py and set "EXTERNAL_ENC" to "None". This will prevent importing the WiringPi & spidev, which otherwise will cause an error. 

## Work with DAnTE

#### 0. Preperation

###### 0.0 Robot.py

###### 0.1 Calibration

Perform calibration before your first run after open-box, and then after every repair/maintemance, or as prompt during usage. To calibrate, cd into ./DAnTE_V2/ directory and run calibration.py(assuming python3):
```bash
python3 -m Play.calibration
```

Then follow instructions as prompt. You can select to calibrate just one single finger(not yet ready) or the whole hand.

The calibration checks the integrity of the system and sets zero references for all actuators. Then it looks for the range of motion for all the fingers or the selected finger. Calibration results will be writen into ../Settings/initials.txt for initialization reference.

All actuators involved get pinged at first when calibration. If there is an communication problem, a 4-bit error code will be generated, with each bit representing a device:

| bit  | Device |
| ---- | -------|
| 3  | PALM  |
| 2  | THUMB  |
| 1  | INDEX_M  |
| 0  | INDEX  |
 
###### 0.2 Initialization

Initialization is a function defined in the robot_controller module. Run initialization first after booting the system. Make sure system is started with ```start_robot()``` function.

The initialization checks if there is any discrepancy between the settings in the actuators registor and the data of the robot instance, then it quickly run all fingers through their rang of motion to make sure nothing is stuck. System will not function without passing initialization.

Initialization example:
```
from Settings.Robot import *
from Play.robot_controller import RobotController

rc = RobotController(robot=DAnTE)
rc.start_robot()
rc.initialization()
```

All actuators get pinged with start_robot(), and the calibration file initials.txt gets checked. If there is any problem, a 5-bit error code will be generated, with each bit representing:

| bit  | Device |
| ---- | -------|
| 3  | Calibration File  |
| 3  | PALM  |
| 2  | THUMB  |
| 1  | INDEX_M  |
| 0  | INDEX  |

There is an abnormal code for initialization, should there be anything wrong. The abnormal code is a list of four(4) 5-bit code, with each code representing a device:

| bit  | Device |
| ---- | -------|
| 3  | PALM  |
| 2  | THUMB  |
| 1  | INDEX_M  |
| 0  | INDEX  |

and each bit representing an error:

| bit  | Error |
| ---- | -------|
| 4  | External encoder offset discrepancy |
| 3  | Failed to travel to home |
| 2  | Failed to fully close |
| 1  | Position out of range |
| 0  | home_offset abnormal |

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

Before you can do anything with DAnTE, you must start the system with ```start_robot()``` function:
```
rc.start_robot()
```
This function pings all actuators, then reads the initals.txt and populate the variables of the robot instance.

###### 1.1 Grab

Use ```grab(gesture, mode, \**options)``` function for object grabbing. 

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

Use ```release(release_mode, \*hold_stiffness)``` function for releasing.

This function instructs DAnTE to release, with three different modes to be specified in release_mode:
- change-to-(H)old = change to hold mode, with the hold_stiffness as specified or go with default (Settings/Macros_DAnTE)
- (L)et-go = release just a little bit so that the object can escape
- (F)ul-release = fingers reset

###### 1.3 Error

And error code will be generated if there is any thing wrong during these operations. The error code and their meanings are:

| code | error |
| ---- | -------|
| 1  | Timeout  |
| 2  | User interruption  |
| 3  | Initialization  |
| 9  | Invalid input  |

#### 2. Tuning

Tune DAnTE via Macros_DAnTE, rarely need to modify Contants_DAnTE.

#### 3. Forward Kinematics
All forward kinematics related basic functions are included in the module Forward_Kinematics/forward_kin.py, and are integrated into Play.robot_controller.py
###### 3.1 Update finger joint angles
This is the first step to update robot with current kinematics. To do so, call ```update_angles()```

This will populate all all finger.angles in the robot instance will present values. The form of this attribute is a list of finger phalanx angles: [alpha, beta, gamma, delta]
###### 3.2 Update finger forward kinematics
Call ```forward_kinematics(\*finger)``` with finger option to calculate the forward kinematics of a finger or all fingers based on the current values stored in finger.angles. The result will populate the corresponding finger(s)' forward kinematics attribute finger.joint_locations. The form of this attribute is an numpy array of 3D joint_locations = [MCP; PIP; DIP; Tip]

Specify the finger to be updated with the \*finger option. 

For example, to updated INDEX:
```rc.forward_kinematics(INDEX)```
If no option specified, all fingers get updated.

Note that the forward kinematics will be calculated based on the present values in finger.angles so make sure to update finger angles first.

###### 3.3 Visualization
```visualization()``` function will plot the robot using the forward kinematics stored in finger.joint_locations.

## Beta Functions

- Full hand kinematics and object size/shape estimation after grabbing.

## Future work

- Interactive operation synced with operator with force


