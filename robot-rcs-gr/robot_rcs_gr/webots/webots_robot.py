import os
import pygame
import math
import numpy
import torch
from enum import Enum
import matplotlib.pyplot as plt

from controller import InertialUnit
from controller import Robot
from controller import Keyboard
from controller import Supervisor
from controller import Node


class WebotsRobot:
    def __init__(self, sim_dt):

        self.robot_name = "Robot"

        self.sim_dt = sim_dt

        self.webots_robot_intf = None

        self.node_base = None
        self.base_target_height = 0.0

        self.num_of_legs = 0

        self.num_of_links = 0
        self.links = []
        self.links_name = []

        self.num_of_joints = 0
        self.joints = []
        self.joints_name = []
        self.joints_kp = []
        self.joints_ki = []
        self.joints_kd = []

        self.num_of_joint_position_sensors = 0
        self.joint_position_sensors = []
        self.joint_position_sensors_name = []

        self.num_of_imus = 0
        self.imus = []
        self.imus_name = []

        self.num_of_gyros = 0
        self.gyros = []
        self.gyros_name = []

        self.num_of_accelerometers = 0
        self.accelerometers = []
        self.accelerometers_name = []

        # measured variables
        self.base_measured_xyz_to_world = None
        self.base_measured_rpy_to_world = None
        self.base_measured_quat_to_world = None
        self.base_measured_xyz_vel_to_world = None
        self.base_measured_rpy_vel_to_world = None

        self.base_measured_xyz_vel_to_self = None
        self.base_measured_rpy_vel_to_self = None

        self.imu_measured_quat_to_world = None
        self.gyro_measured_rpy_vel_to_self = None

        self.joint_measured_position_value = None
        self.joint_measured_velocity_value = None

        self.joint_position_sensor_value = None

        # sim
        self.joint_pd_control_target_to_sim = None

        # pd control
        self.joint_pd_control_target = numpy.zeros(self.num_of_joints)
        self.joint_pd_control_output = numpy.zeros(self.num_of_joints)

        self.joint_pd_control_target_buffer = []
        self.joint_pd_control_target_delay = 0

        for i in range(self.joint_pd_control_target_delay + 1):
            self.joint_pd_control_target_buffer.append(numpy.zeros(self.num_of_joints))

        self.joint_pd_control_kp = numpy.array([])
        self.joint_pd_control_kd = numpy.array([])
        self.joint_pd_control_max = numpy.array([])
        self.joint_pd_control_min = numpy.array([])

    def prepare(self):
        self.webots_robot_intf = Supervisor()

        self.node_base = self.webots_robot_intf.getFromDef(self.robot_name)

        for name in self.links_name:
            self.links.append(self.webots_robot_intf.getFromDef(name))

        print("links = \n", self.links)

        for name in self.joints_name:
            self.joints.append(self.webots_robot_intf.getDevice(name))

        print("joints = \n", self.joints)

        # for i in range(len(self.joints)):
        #     joint = self.joints[i]
        #     joint_kp = self.joints_kp[i]
        #     joint_ki = self.joints_ki[i]
        #     joint_kd = self.joints_kd[i]
        #
        #     joint.setControlPID(joint_kp, joint_ki, joint_kd)

        for name in self.joint_position_sensors_name:
            self.joint_position_sensors.append(self.webots_robot_intf.getDevice(name))

        print("joint_position_sensors = \n", self.joint_position_sensors)

        for name in self.imus_name:
            self.imus.append(self.webots_robot_intf.getDevice(name))

        print("imus = \n", self.imus)

        for name in self.gyros_name:
            self.gyros.append(self.webots_robot_intf.getDevice(name))

        print("gyros = \n", self.gyros)

        for name in self.accelerometers_name:
            self.accelerometers.append(self.webots_robot_intf.getDevice(name))

        print("accelerometers = \n", self.accelerometers)

        # measured value
        self.joint_measured_position_value = numpy.zeros(self.num_of_joints)
        self.joint_measured_position_value_last = numpy.zeros(self.num_of_joints)
        self.joint_measured_velocity_value = numpy.zeros(self.num_of_joints)
        self.joint_measured_velocity_value_last = numpy.zeros(self.num_of_joints)
        self.joint_measured_force_value = numpy.zeros(self.num_of_joints)
        self.joint_measured_torque_value = numpy.zeros(self.num_of_joints)
        self.joint_position_sensors_value = numpy.zeros(self.num_of_joint_position_sensors)

        print("tasks = \n", self.tasks)

    def enable(self):
        for joint in self.joints:
            joint.enableForceFeedback(self.sim_dt)
            joint.enableTorqueFeedback(self.sim_dt)

        for joint_position_sensor in self.joint_position_sensors:
            # The sampling_period argument specifies the sampling period of the sensor and is expressed in milliseconds.
            joint_position_sensor.enable(self.sim_dt)

        for imu in self.imus:
            imu.enable(self.sim_dt)

        for gyro in self.gyros:
            gyro.enable(self.sim_dt)

        for accelerater in self.accelerometers:
            accelerater.enable(self.sim_dt)

    def control_loop_update_robot_state(self):

        # 读取 robot 位置
        self.base_measured_xyz_to_world = numpy.array(self.node_base.getPosition())
        self.base_measured_rm_to_world = numpy.array(self.node_base.getOrientation())
        self.base_measured_xyz_vel_to_world = numpy.array(self.node_base.getVelocity()[0:3])
        self.base_measured_rpy_vel_to_world = numpy.array(self.node_base.getVelocity()[3:6])

        # 读取传感器数据
        imu_base_ang_value_in_rpy = self.imus[0].getRollPitchYaw()
        imu_base_ang_value_in_quat = self.imus[0].getQuaternion()

        self.imu_measured_quat_to_world = imu_base_ang_value_in_quat

        gyro_base_ang_vel_value = self.gyros[0].getValues()

        self.gyro_measured_rpy_vel_to_self = numpy.array(gyro_base_ang_vel_value)

        # 读取电机数据
        for i in range(self.num_of_joints):
            self.joint_measured_force_value[i] = self.joints[i].getForceFeedback()
            self.joint_measured_torque_value[i] = self.joints[i].getTorqueFeedback()

        # 读取关节位置传感器数据
        for i in range(self.num_of_joint_position_sensors):
            self.joint_position_sensor_value = round(self.joint_position_sensors[i].getValue(), 8)  # keep 4 value after .
            self.joint_position_sensors_value[i] = self.joint_position_sensor_value

        self.joint_measured_position_value_last = numpy.array(self.joint_measured_position_value)  # 记录上一次的关节位置
        self.joint_measured_position_value = numpy.array(self.joint_position_sensors_value)

        # 计算关节速度
        self.joint_measured_velocity_value_last = numpy.array(self.joint_measured_velocity_value)
        self.joint_measured_velocity_value = \
            (self.joint_measured_position_value - self.joint_measured_position_value_last) / (self.sim_dt * 0.001)

    def control_loop_algorithm(self):
        pass

    def control_loop_output(self):

        # Jason 2024-02-27:
        # add delay to the joint_pd_control_target
        self.joint_pd_control_target_buffer = self.joint_pd_control_target_buffer[1:]
        self.joint_pd_control_target_buffer.append(self.joint_pd_control_target)

        # self.joint_pd_control_target = self.joint_pd_control_target_buffer[0]  # use the first element of the buffer
        self.joint_pd_control_target = self.joint_pd_control_target

        # PD Control
        self.joint_pd_control_output = \
            self.joint_pd_control_kp * (self.joint_pd_control_target
                                        - self.joint_measured_position_value) \
            - self.joint_pd_control_kd * (self.joint_measured_velocity_value)

        self.joint_pd_control_output = \
            numpy.clip(self.joint_pd_control_output,
                       self.joint_pd_control_min,
                       self.joint_pd_control_max)

        # output
        for i in range(self.num_of_joints):
            if self.task_algorithm_model.flag_joint_pd_torque_control[i] is True:
                self.joints[i].setTorque(self.joint_pd_control_output[i])

            if self.task_algorithm_model.flag_joint_position_control[i] is True:
                self.joints[i].setPosition(self.joint_pd_control_target[i])
