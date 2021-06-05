#!usr/bin/env python
__author__ = "X Zhang"
__email__ = "xzhang@westwoodrobotics.net"
__copyright__ = "Copyright 2020 Westwood Robotics Corp."
__date__ = "Feb 14, 2020"
__version__ = "0.1.0"
__status__ = "Beta"

'''
This module is used to communicate with the BEAR modules on the DAnTE platform
'''

from pybear import Manager
import math
from Settings.Robot import *
import pdb


class MotorController(object):

    def __init__(self, baudrate, port):
        self.baudrate = baudrate
        self.port = port
        self.pbm = Manager.BEAR(port=self.port, baudrate=self.baudrate)
        # self.motor_count = FingerDataStructure.finger_count

    def init_driver(self, m_id):
        """
        Initialize all drivers with preset PID and safe limits.


        :return: None

        """
        self.pbm.set_p_gain_id((m_id, IQID_P))
        self.pbm.set_p_gain_iq((m_id, IQID_P))
        self.pbm.set_i_gain_id((m_id, IQID_I))
        self.pbm.set_i_gain_iq((m_id, IQID_I))
        self.pbm.set_d_gain_id((m_id, IQID_D))
        self.pbm.set_d_gain_iq((m_id, IQID_D))

        # Velocity PID
        self.pbm.set_p_gain_velocity((m_id, VEL_P))
        self.pbm.set_i_gain_velocity((m_id, VEL_I))
        self.pbm.set_d_gain_velocity((m_id, VEL_D))

        # Position PID
        self.pbm.set_p_gain_position((m_id, POS_P))
        self.pbm.set_i_gain_position((m_id, POS_I))
        self.pbm.set_d_gain_position((m_id, POS_D))

        # Clear Direct Force PID
        self.pbm.set_p_gain_force((m_id, 0))
        self.pbm.set_i_gain_force((m_id, 0))
        self.pbm.set_d_gain_force((m_id, 0))

        # Set safe iq_max and velocity_max
        self.pbm.set_limit_iq_max((m_id, IQ_MAX_INIT))
        self.pbm.set_limit_velocity_max((m_id, VEL_MAX_INIT))

        print("Motor driver initialized for Motor:%d." % m_id)

    def init_driver_all(self):
        """
        Initialize all drivers with preset PID and safe limits.

        :return: None
        """
        # Set safe iq_max and velocity_max
        for f_id in DAnTE.finger_ids:
            self.init_driver(f_id)

        print("All motor drivers initialized.")

    def set_mode(self, m_id, mode):
        """
        Set the single actuator into a desired mode.

        :param int m_id: Motor ID
        :param str mode:
        """
        if mode == 'position':
            m = 2
        elif mode == 'velocity':
            m = 1
        elif mode == 'torque':
            m = 0
        elif mode == 'force':
            m = 3
        else:
            print("Invalid mode.")
            return

        self.pbm.set_mode((m_id, m))
        while self.pbm.get_mode(m_id)[0][0] != m:
            self.pbm.set_mode((m_id, m))
            # print("Trying hard to set_mode")
            # time.sleep(0.1)

    def set_mode_all(self, mode):
        """
        Set the actuator into a desired mode.
        Enforced

        :param str mode: Desired mode. (torque, velocity, position, force)
        """
        if mode == 'position':
            m = 2
        elif mode == 'velocity':
            m = 1
        elif mode == 'torque':
            m = 0
        elif mode == 'force':
            m = 3
        else:
            print("Invalid mode.")
            return
        self.pbm.set_mode((BEAR_THUMB, m), (BEAR_INDEX, m), (BEAR_INDEX_M, m))
        # Check for change
        status = [self.pbm.get_mode(BEAR_THUMB)[0][0], self.pbm.get_mode(BEAR_INDEX)[0][0], self.pbm.get_mode(BEAR_INDEX_M)[0][0]]
        checksum = sum([i == m for i in status])
        while checksum != 3:
            self.pbm.set_mode((BEAR_THUMB, m), (BEAR_INDEX, m), (BEAR_INDEX_M, m))
            status = [self.pbm.get_mode(BEAR_THUMB)[0][0], self.pbm.get_mode(BEAR_INDEX)[0][0], self.pbm.get_mode(BEAR_INDEX_M)[0][0]]
            checksum = sum([i == m for i in status])
            print("Trying hard to set_mode_all")
            print(status)
            print(checksum)

    def torque_enable(self, m_id, val):
        """
        Torque enable.

        :param m_id:
        :param int val: Enable/disable torque. (0: disable, 1: enable)
        """
        self.pbm.set_torque_enable((m_id, val))
        while bool(self.pbm.get_torque_enable(m_id)[0][0]) != bool(val):
            self.pbm.set_torque_enable((m_id, val))
        if val:
            return True
        else:
            return False

    def torque_enable_all(self, val):
        """
        Torque enable.

        :param int val: Enable/disable torque. (0: disable, 1: enable)
        """

        # Check for change
        status = [bool(self.pbm.get_torque_enable(BEAR_THUMB)[0][0]), bool(self.pbm.get_torque_enable(BEAR_INDEX)[0][0]),
                  bool(self.pbm.get_torque_enable(BEAR_INDEX_M)[0][0])]
        checksum = sum([i == val for i in status])
        while checksum != 3:
            self.pbm.set_torque_enable((BEAR_THUMB, val), (BEAR_INDEX, val), (BEAR_INDEX_M, val))
            status = [bool(self.pbm.get_torque_enable(BEAR_THUMB)[0][0]), bool(self.pbm.get_torque_enable(BEAR_INDEX)[0][0]),
                      bool(self.pbm.get_torque_enable(BEAR_INDEX_M)[0][0])]
            checksum = sum([i == val for i in status])

        if val:
            return [True, True, True]
        else:
            return [False, False, False]

    def get_enable_all(self):
        """
        Get torque enable status of all three BEARs.

        :return: [INDEX enable, INDEX_M enable, THUMB enable]
        """
        enable = self.pbm.get_torque_enable(BEAR_INDEX, BEAR_INDEX_M, BEAR_THUMB)
        return [bool(i[0]) for i in enable]

    def damping_mode_all(self):
        """
        Sets the joints into damping mode by reducing the joint limits to 0 so that any position triggers damping mode.
        """
        self.pbm.set_limit_position_max((BEAR_THUMB, 0.0), (BEAR_INDEX, 0.0), (BEAR_INDEX_M, 0.0))
        self.pbm.set_limit_position_min((BEAR_THUMB, 0.0), (BEAR_INDEX, 0.0), (BEAR_INDEX_M, 0.0))

    def damping_mode(self, m_id):
        """
        Sets the joints into damping mode by reducing the joint limits to 0 so that any position triggers damping mode.
        """
        self.pbm.set_limit_position_max((m_id, 0.0))
        self.pbm.set_limit_position_min((m_id, 0.0))

    def damping_release_all(self, **limits):
        """
        Releases the joints from damping mode by restoring the joint limits to args or default
        Use kwargs limit_min and limit_max to specify desired limits, otherwise default limits adopted.
        """
        limit_max = 8*math.pi
        limit_min = -8*math.pi
        if len(limits):
            for limit, value in limits.items():
                if limit == 'limit_min':
                    limit_min = value
                elif limit == 'limit_max':
                    limit_max = value
                else:
                    print('%s is not recognized. Use limit_min and limit_max only.' % limit)

        self.pbm.set_limit_position_max((BEAR_THUMB, limit_max), (BEAR_INDEX, limit_max), (BEAR_INDEX_M, limit_max))
        self.pbm.set_limit_position_min((BEAR_THUMB, limit_min), (BEAR_INDEX, limit_min), (BEAR_INDEX_M, limit_min))
        #  Disable and enable to clear error
        self.torque_enable_all(0)
        self.torque_enable_all(1)

    def damping_release(self, m_id, **limits):
        """
        Releases the joint from damping mode by restoring the joint limits to args or default
        Use kwargs limit_min and limit_max to specify desired limits, otherwise default limits adopted.
        """
        limit_max = 8 * math.pi
        limit_min = -8 * math.pi
        if len(limits):
            for limit, value in limits.items():
                if limit == 'limit_min':
                    limit_min = value
                elif limit == 'limit_max':
                    limit_max = value
                else:
                    print('%s is not recognized. Use limit_min and limit_max only.' % limit)
        self.pbm.set_limit_position_max((m_id, limit_max))
        self.pbm.set_limit_position_min((m_id, limit_min))
        #  Disable and enable to clear error
        self.pbm.set_torque_enable((m_id, 0))
        self.pbm.set_torque_enable((m_id, 1))

    def get_present_status_all(self):
        """
        Get present status of all fingers
        """
        # TODO: replace with bulk_comm function
        # info_all = self.pbm.bulk_read((BEAR_THUMB, BEAR_INDEX, BEAR_INDEX),
        #                              ('present_position', 'present_velocity', 'present_iq'))
        info_all = self.pbm.get_bulk_status((BEAR_INDEX, 'present_position', 'present_velocity', 'present_iq'),
                                            (BEAR_INDEX_M, 'present_position', 'present_velocity', 'present_iq'),
                                            (BEAR_THUMB, 'present_position', 'present_velocity', 'present_iq'))
        return info_all

    def grab_loop_comm(self, gesture, goal_pos, goal_iq):
        """
        Main grab loop communication function using bulk_read_write.

        :param str gesture: The present gesture of DAnTE
        :param list goal_pos: The goal position command to send
        :param list goal_iq: The goal iq command to send
        :return: present info [[pos, vel, iq], ...]
        :rtype: list of list
        """
        command = []
        for i in range(len(goal_pos)):
            command.append([goal_pos[i], goal_iq[i]])
        info_all = None
        while info_all is None:
            if gesture == 'I':
                # Pinch mode, not using THUMB
                info_all = self.pbm.bulk_read_write([BEAR_INDEX, BEAR_INDEX_M],
                                                    ['present_position', 'present_velocity', 'present_iq'],
                                                    ['goal_position', 'goal_iq'],
                                                    command, error_mode=1)
            else:
                info_all = self.pbm.bulk_read_write([BEAR_INDEX, BEAR_INDEX_M, BEAR_THUMB],
                                                    ['present_position', 'present_velocity', 'present_iq'],
                                                    ['goal_position', 'goal_iq'],
                                                    command, error_mode=1)
        return info_all

    def grab_loop_comm_all(self, gesture, goal_pos, goal_vel, goal_iq):
        """
        Main grab loop communication function using bulk_read_write.

        :param str gesture: The present gesture of DAnTE
        :param list goal_pos: The goal position command to send
        :param list goal_vel: The goal_velocity command to send
        :param list goal_iq: The goal iq command to send
        :return: present info [[pos, vel, iq], ...]
        :rtype: list of list
        """
        command = []
        for i in range(len(goal_pos)):
            command.append([goal_pos[i], goal_vel[i], goal_iq[i]])
        info_all = None
        while info_all is None:
            if gesture == 'I':
                # Pinch mode, not using THUMB
                info_all = self.pbm.bulk_read_write([BEAR_INDEX, BEAR_INDEX_M],
                                                    ['present_position', 'present_velocity', 'present_iq'],
                                                    ['goal_position', 'goal_velocity', 'goal_iq'],
                                                    command, error_mode=1)
            else:
                info_all = self.pbm.bulk_read_write([BEAR_INDEX, BEAR_INDEX_M, BEAR_THUMB],
                                                    ['present_position', 'present_velocity', 'present_iq'],
                                                    ['goal_position', 'goal_velocity', 'goal_iq'],
                                                    command, error_mode=1)
        return info_all

    def grab_loop_confirm_goals(self, gesture, goal_pos, goal_vel, goal_iq):
        """
        Confirm all goals

        :param str gesture: The present gesture of DAnTE
        :param list goal_pos: The goal position command to send
        :param list goal_vel: The goal_velocity command to send
        :param list goal_iq: The goal iq command to send
        """
        goals = None
        rtn = None
        if gesture == 'I':
            checksum = round(sum(goal_pos[:2]+goal_vel[:2]+goal_iq[:2]), 5)
        else:
            checksum = round(sum(goal_pos + goal_vel + goal_iq), 5)
        command = []
        for i in range(len(goal_pos)):
            command.append([goal_pos[i], goal_vel[i], goal_iq[i]])
        while goals != checksum:
            while rtn is None:
                if gesture == 'I':
                    # Pinch mode, not using THUMB
                    rtn = self.pbm.bulk_read_write([BEAR_INDEX, BEAR_INDEX_M],
                                                   ['goal_position', 'goal_velocity', 'goal_iq'],
                                                   ['goal_position', 'goal_velocity', 'goal_iq'], command, error_mode=1)
                else:
                    rtn = self.pbm.bulk_read_write([BEAR_INDEX, BEAR_INDEX_M, BEAR_THUMB],
                                                   ['goal_position', 'goal_velocity', 'goal_iq'],
                                                   ['goal_position', 'goal_velocity', 'goal_iq'], command, error_mode=1)
            goals = [sum(data[0]) for data in rtn]
            goals = round(sum(goals), 5)

    def grab_loop_comm_velocity(self, gesture, goal_velocity):
        """
        Main grab loop communication function using bulk_read_write.
        Only write goal_velocity

        :param str gesture: The present gesture of DAnTE
        :param list goal_velocity: The goal_velocity command to send
        :return: present info [[pos, vel, iq], ...]
        :rtype: list of list
        """
        command = []
        for i in range(len(goal_velocity)):
            command.append([goal_velocity[i]])
        info_all = None
        while info_all is None:
            if gesture == 'I':
                # Pinch mode, not using THUMB
                info_all = self.pbm.bulk_read_write([BEAR_INDEX, BEAR_INDEX_M],
                                                    ['present_position', 'present_velocity', 'present_iq'],
                                                    ['goal_velocity'],
                                                    command, error_mode=1)
            else:
                info_all = self.pbm.bulk_read_write([BEAR_INDEX, BEAR_INDEX_M, BEAR_THUMB],
                                                    ['present_position', 'present_velocity', 'present_iq'],
                                                    ['goal_velocity'],
                                                    command, error_mode=1)
        return info_all

    def set_goal_iq(self, gesture, goal_iq):
        """
        Set goal iq to currently in-use fingers using bulk_write.

        :param str gesture: The present gesture of DAnTE
        :param list goal_iq: The goal iq command to send
        """
        command = []
        for i in range(len(goal_iq)):
            command.append([goal_iq[i]])
        if gesture == 'I':
            # Pinch mode, not using THUMB
            self.pbm.bulk_write([BEAR_INDEX, BEAR_INDEX_M], ['goal_iq'], command)
        else:
            self.pbm.bulk_write([BEAR_INDEX, BEAR_INDEX_M, BEAR_THUMB], ['goal_iq'], command)

    def set_goal_position(self, gesture, goal_pos):
        """
        Set goal position to currently in-use fingers using bulk_write.

        :param str gesture: The present gesture of DAnTE
        :param list goal_pos: The goal position command to send
        """
        command = []
        for i in range(len(goal_pos)):
            command.append([goal_pos[i]])
        if gesture == 'I':
            # Pinch mode, not using THUMB
            self.pbm.bulk_write([BEAR_INDEX, BEAR_INDEX_M], ['goal_position'], command)
        else:
            self.pbm.bulk_write([BEAR_INDEX, BEAR_INDEX_M, BEAR_THUMB], ['goal_position'], command)

    def get_present_status_index(self):
        """
        Get present status of all fingers
        """
        # TODO: replace with bulk_comm function
        # info_all = self.pbm.bulk_read((BEAR_THUMB, BEAR_INDEX, BEAR_INDEX),
        #                              ('present_position', 'present_velocity', 'present_iq'))
        info_all = self.pbm.get_bulk_status((BEAR_INDEX, 'present_position', 'present_velocity', 'present_iq'),
                                            (BEAR_INDEX_M, 'present_position', 'present_velocity', 'present_iq'))
        return info_all

    def get_present_status(self, m_id):
        """
        Get present status of all fingers
        """
        # TODO: replace with bulk_comm function
        info = self.pbm.get_bulk_status((m_id, 'present_position', 'present_velocity', 'present_iq'))

        return info

    def get_present_position_all(self):
        """
        Get present position of all fingers
        """
        info_all = None
        while info_all is None:
            info_all = self.pbm.bulk_read([BEAR_INDEX, BEAR_INDEX_M, BEAR_THUMB], ['present_position'], error_mode=1)
        info_all = [i[0][0] for i in info_all]
        return info_all

    def save_congif_all(self):
        """
        Save config for all
        """
        self.pbm.save_config(BEAR_THUMB)
        self.pbm.save_config(BEAR_INDEX)
        self.pbm.save_config(BEAR_INDEX_M)

