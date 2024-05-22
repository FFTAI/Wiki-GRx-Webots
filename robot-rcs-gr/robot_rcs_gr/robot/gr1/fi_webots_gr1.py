import numpy
from enum import Enum

from controller import InertialUnit
from controller import Robot
from controller import Keyboard
from controller import Supervisor
from controller import Node


class WebotsGR1(WebotsRobot):
    def __init__(self, sim_dt):
        super().__init__(sim_dt=sim_dt)

        self.robot_name = "GR1"

        self.base_target_height = 0.90

        self.num_of_legs = 2

        self.num_of_links = 3 * 4
        self.links = []
        self.links_name = [
            "1_thigh_roll", "1_thigh_pitch", "1_shank",
            "3_thigh_roll", "3_thigh_pitch", "3_shank",
            "5_thigh_roll", "5_thigh_pitch", "5_shank",
            "7_thigh_roll", "7_thigh_pitch", "7_shank",
        ]

        self.num_of_joints = 3 * 4
        self.joints = []
        self.joints_name = [
            "1_hip_roll", "1_hip_pitch", "1_knee",
            "3_hip_roll", "3_hip_pitch", "3_knee",
            "5_hip_roll", "5_hip_pitch", "5_knee",
            "7_hip_roll", "7_hip_pitch", "7_knee",
        ]
        self.joints_kp = [
            0, 0, 0,
            0, 0, 0,
            0, 0, 0,
            0, 0, 0,
        ]
        self.joints_ki = [
            0, 0, 0,
            0, 0, 0,
            0, 0, 0,
            0, 0, 0,
        ]
        self.joints_kd = [
            0, 0, 0,
            0, 0, 0,
            0, 0, 0,
            0, 0, 0,
        ]

        self.num_of_joint_position_sensors = 3 * 4
        self.joint_position_sensors = []
        self.joint_position_sensors_name = [
            "1_hip_roll_sensor", "1_hip_pitch_sensor", "1_knee_sensor",
            "3_hip_roll_sensor", "3_hip_pitch_sensor", "3_knee_sensor",
            "5_hip_roll_sensor", "5_hip_pitch_sensor", "5_knee_sensor",
            "7_hip_roll_sensor", "7_hip_pitch_sensor", "7_knee_sensor",
        ]

        self.num_of_imus = 1
        self.imus = []
        self.imus_name = [
            "inertial_unit"
        ]

        self.num_of_gyros = 1
        self.gyros = []
        self.gyros_name = [
            "gyro"
        ]

        self.num_of_accelerometers = 1
        self.accelerometers = []
        self.accelerometers_name = [
            "accelerometer"
        ]

        # measured variables
        self.base_measured_rpy_to_world = None
        self.base_measured_quat_to_world = None
        self.base_measured_xyz_vel_to_self = None
        self.base_measured_rpy_vel_to_self = None

        self.joint_position_sensor_value = None

        # pd control
        self.joint_pd_control_target = numpy.zeros(self.num_of_joints)
        self.joint_pd_control_output = numpy.zeros(self.num_of_joints)

        self.joint_pd_control_target_buffer = []
        self.joint_pd_control_target_delay = 0

        for i in range(self.joint_pd_control_target_delay + 1):
            self.joint_pd_control_target_buffer.append(numpy.zeros(self.num_of_joints))

        self.joint_pd_control_kp = numpy.array([
            200.0, 200.0, 200.0,  # left front leg(3)
            200.0, 200.0, 200.0,  # left back leg(3)
            200.0, 200.0, 200.0,  # right back leg(3)
            200.0, 200.0, 200.0,  # right front leg(3)
        ])
        self.joint_pd_control_kd = numpy.array([
            10.0, 10.0, 10.0,  # left front leg(3)
            10.0, 10.0, 10.0,  # left back leg(3)
            10.0, 10.0, 10.0,  # right back leg(3)
            10.0, 10.0, 10.0,  # right front leg(3)
        ])
        self.joint_pd_control_max = numpy.array([
            150.0, 150.0, 150.0,  # left front leg(3)
            150.0, 150.0, 150.0,  # left back leg(3)
            150.0, 150.0, 150.0,  # right back leg(3)
            150.0, 150.0, 150.0,  # right front leg(3)
        ])
        self.joint_pd_control_min = numpy.array([
            -150.0, -150.0, -150.0,  # left front leg(3)
            -150.0, -150.0, -150.0,  # left back leg(3)
            -150.0, -150.0, -150.0,  # right back leg(3)
            -150.0, -150.0, -150.0,  # right front leg(3)
        ])
