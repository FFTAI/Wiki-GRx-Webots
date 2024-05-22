import os
import numpy
import torch
from enum import Enum

from controller import InertialUnit
from controller import Robot
from controller import Keyboard
from controller import Supervisor
from controller import Node


class WebotsGR1Simple(WebotsRobot):
    def __init__(self, sim_dt):
        super().__init__(sim_dt=sim_dt)

        self.robot_name = "GR1Simple"

        self.base_target_height = 0.90

        self.num_of_legs = 2

        self.num_of_links = 5 + 5
        self.links_name = [
            # left leg
            "l_thigh_roll", "l_thigh_yaw", "l_thigh_pitch", "l_shank_pitch", "l_foot_pitch",
            # right leg
            "r_thigh_roll", "r_thigh_yaw", "r_thigh_pitch", "r_shank_pitch", "r_foot_pitch",
        ]

        self.num_of_joints = 5 + 5
        self.joints_name = [
            # left leg
            "l_hip_roll", "l_hip_yaw", "l_hip_pitch", "l_knee_pitch", "l_ankle_pitch",
            # right leg
            "r_hip_roll", "r_hip_yaw", "r_hip_pitch", "r_knee_pitch", "r_ankle_pitch",
        ]
        self.joints_kp = [
            0, 0, 0, 0, 0,
            0, 0, 0, 0, 0,
        ]
        self.joints_ki = [
            0, 0, 0, 0, 0,
            0, 0, 0, 0, 0,
        ]
        self.joints_kd = [
            0, 0, 0, 0, 0,
            0, 0, 0, 0, 0,
        ]

        self.num_of_joint_position_sensors = 5 + 5
        self.joint_position_sensors_name = [
            # left leg
            "l_hip_roll_sensor", "l_hip_yaw_sensor", "l_hip_pitch_sensor", "l_knee_pitch_sensor", "l_ankle_pitch_sensor",
            # right leg
            "r_hip_roll_sensor", "r_hip_yaw_sensor", "r_hip_pitch_sensor", "r_knee_pitch_sensor", "r_ankle_pitch_sensor",
        ]

        self.num_of_imus = 1
        self.imus_name = [
            "inertial unit"
        ]

        self.num_of_gyros = 1
        self.gyros_name = [
            "gyro"
        ]

        self.num_of_accelerometers = 1
        self.accelerometers_name = [
            "accelerometer"
        ]

        # pd control
        self.joint_pd_control_target = numpy.zeros(self.num_of_joints)
        self.joint_pd_control_output = numpy.zeros(self.num_of_joints)

        self.joint_pd_control_target_buffer = []
        self.joint_pd_control_target_delay = 0

        for i in range(self.joint_pd_control_target_delay + 1):
            self.joint_pd_control_target_buffer.append(numpy.zeros(self.num_of_joints))

        self.joint_pd_control_kp = numpy.array([
            251.625, 362.5214, 200.0, 200.0, 10.9885,  # left leg(5)
            251.625, 362.5214, 200.0, 200.0, 10.9885,  # right leg(5)
        ])
        self.joint_pd_control_kd = numpy.array([
            14.72, 10.0833, 11.0, 11.0, 0.5991,  # left leg(5)
            14.72, 10.0833, 11.0, 11.0, 0.5991,  # right leg(5)
        ])
        self.joint_pd_control_max = numpy.array([
            60.0, 45.0, 130.0, 130.0, 16.0,  # left leg(5)
            60.0, 45.0, 130.0, 130.0, 16.0,  # right leg(5)
        ])
        self.joint_pd_control_min = numpy.array([
            -60.0, -45.0, -130.0, -130.0, -16.0,  # left leg(5)
            -60.0, -45.0, -130.0, -130.0, -16.0,  # right leg(5)
        ])
