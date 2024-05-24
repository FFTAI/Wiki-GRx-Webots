import numpy

from robot_rcs_gr.webots.webots_robot import WebotsRobot


class WebotsGR1T2(WebotsRobot):
    def __init__(self, sim_dt):
        super().__init__(sim_dt=sim_dt)

        self.robot_name = "GR1T2"

        self.base_target_height = 0.90

        self.num_of_legs = 2

        self.num_of_links = 6 + 6 + 3 + 3 + 7 + 7
        self.links = []
        self.links_name = [
            # left leg
            "l_thigh_roll", "l_thigh_yaw", "l_thigh_pitch", "l_shank_pitch", "l_foot_pitch", "l_foot_roll",
            # right leg
            "r_thigh_roll", "r_thigh_yaw", "r_thigh_pitch", "r_shank_pitch", "r_foot_pitch", "r_foot_roll",
            # waist
            "waist_pitch", "waist_roll", "waist_yaw",
            # head
            "head_pitch", "head_yaw", "head_roll",
            # left arm
            "l_upper_arm_pitch", "l_upper_arm_roll", "l_upper_arm_yaw", "l_lower_arm_pitch", "l_hand_pitch", "l_hand_roll", "l_hand_yaw",
            # right arm
            "r_upper_arm_pitch", "r_upper_arm_roll", "r_upper_arm_yaw", "r_lower_arm_pitch", "r_hand_pitch", "r_hand_roll", "r_hand_yaw",
        ]

        self.num_of_joints = 6 + 6 + 3 + 3 + 7 + 7
        self.joints = []
        self.joints_name = [
            # left leg
            "l_hip_roll", "l_hip_yaw", "l_hip_pitch", "l_knee_pitch", "l_ankle_pitch", "l_ankle_roll",
            # right leg
            "r_hip_roll", "r_hip_yaw", "r_hip_pitch", "r_knee_pitch", "r_ankle_pitch", "r_ankle_roll",
            # waist
            "waist_pitch", "waist_roll", "waist_yaw",
            # head
            "head_pitch", "head_yaw", "head_roll",
            # left arm
            "l_upper_arm_pitch", "l_upper_arm_roll", "l_upper_arm_yaw", "l_lower_arm_pitch", "l_hand_pitch", "l_hand_roll", "l_hand_yaw",
            # right arm
            "r_upper_arm_pitch", "r_upper_arm_roll", "r_upper_arm_yaw", "r_lower_arm_pitch", "r_hand_pitch", "r_hand_roll", "r_hand_yaw",
        ]
        self.joints_kp = [
            0, 0, 0, 0, 0, 0,
            0, 0, 0, 0, 0, 0,
            0, 0, 0,
            0, 0, 0,
            0, 0, 0, 0, 0, 0, 0,
            0, 0, 0, 0, 0, 0, 0,
        ]
        self.joints_ki = [
            0, 0, 0, 0, 0, 0,
            0, 0, 0, 0, 0, 0,
            0, 0, 0,
            0, 0, 0,
            0, 0, 0, 0, 0, 0, 0,
            0, 0, 0, 0, 0, 0, 0,
        ]
        self.joints_kd = [
            0, 0, 0, 0, 0, 0,
            0, 0, 0, 0, 0, 0,
            0, 0, 0,
            0, 0, 0,
            0, 0, 0, 0, 0, 0, 0,
            0, 0, 0, 0, 0, 0, 0,
        ]

        self.num_of_joint_position_sensors = 6 + 6 + 3 + 3 + 7 + 7
        self.joint_position_sensors = []
        self.joint_position_sensors_name = [
            # left leg
            "l_hip_roll_sensor", "l_hip_yaw_sensor", "l_hip_pitch_sensor", "l_knee_pitch_sensor", "l_ankle_pitch_sensor", "l_ankle_roll_sensor",
            # right leg
            "r_hip_roll_sensor", "r_hip_yaw_sensor", "r_hip_pitch_sensor", "r_knee_pitch_sensor", "r_ankle_pitch_sensor", "r_ankle_roll_sensor",
            # waist
            "waist_pitch_sensor", "waist_roll_sensor", "waist_yaw_sensor",
            # head
            "head_pitch_sensor", "head_yaw_sensor", "head_roll_sensor",
            # left arm
            "l_upper_arm_pitch_sensor", "l_upper_arm_roll_sensor", "l_upper_arm_yaw_sensor", "l_lower_arm_pitch_sensor", "l_hand_pitch_sensor", "l_hand_roll_sensor", "l_hand_yaw_sensor",
            # right arm
            "r_upper_arm_pitch_sensor", "r_upper_arm_roll_sensor", "r_upper_arm_yaw_sensor", "r_lower_arm_pitch_sensor", "r_hand_pitch_sensor", "r_hand_roll_sensor", "r_hand_yaw_sensor",
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
        # default : stand joint position
        self.joint_pd_control_target = numpy.array([
            0.0, 0.0, -0.2618, 0.5236, -0.2618, 0.0,  # left leg (6)
            0.0, 0.0, -0.2618, 0.5236, -0.2618, 0.0,  # right leg (6)
            0.0, 0.0, 0.0,  # waist (3)
            0.0, 0.0, 0.0,  # head (3)
            0.0, 0.2, 0.0, -0.3, 0.0, 0.0, 0.0,  # left arm (7)
            0.0, -0.2, 0.0, -0.3, 0.0, 0.0, 0.0,  # right arm (7)
        ])
        self.joint_pd_control_output = self.joint_pd_control_target

        self.joint_pd_control_kp = numpy.array([
            251.625, 362.5214, 200.0, 200.0, 10.9885, 0.25,  # left leg(6)
            251.625, 362.5214, 200.0, 200.0, 10.9885, 0.25,  # right leg(6)
            362.5214, 362.5214, 362.5214,  # waist(3)
            10.0, 10.0, 10.0,  # head(3)
            92.85, 92.85, 112.06, 112.06, 10.0, 10.0, 10.0,  # left arm(7)
            92.85, 92.85, 112.06, 112.06, 10.0, 10.0, 10.0,  # right arm(7)
        ])
        self.joint_pd_control_kd = numpy.array([
            14.72, 10.0833, 11.0, 11.0, 0.5991, 0.01,  # left leg(6)
            14.72, 10.0833, 11.0, 11.0, 0.5991, 0.01,  # right leg(6)
            10.0833, 10.0833, 10.0833,  # waist(3)
            1.0, 1.0, 1.0,  # head(3)
            2.575, 2.575, 3.1, 3.1, 1.0, 1.0, 1.0,  # left arm(7)
            2.575, 2.575, 3.1, 3.1, 1.0, 1.0, 1.0,  # right arm(7)
        ])
        self.joint_pd_control_max = numpy.array([
            100.0, 82.5, 150.0, 150.0, 16.0, 8.0,  # left leg(6)
            100.0, 82.5, 150.0, 150.0, 16.0, 8.0,  # right leg(6)
            82.5, 82.5, 82.5,  # waist(3)
            10.0, 10.0, 10.0,  # head(3)
            38.0, 38.0, 30.0, 30.0, 10.0, 10.0, 10.0,  # left arm(7)
            38.0, 38.0, 30.0, 30.0, 10.0, 10.0, 10.0,  # right arm(7)
        ])
        self.joint_pd_control_min = numpy.array([
            -100.0, -82.5, -150.0, -150.0, -16.0, -8.0,  # left leg(6)
            -100.0, -82.5, -150.0, -150.0, -16.0, -8.0,  # right leg(6)
            -82.5, -82.5, -82.5,  # waist(3)
            -10.0, -10.0, -10.0,  # head(3)
            -38.0, -38.0, -30.0, -30.0, -10.0, -10.0, -10.0,  # left arm(7)
            -38.0, -38.0, -30.0, -30.0, -10.0, -10.0, -10.0,  # right arm(7)
        ])
