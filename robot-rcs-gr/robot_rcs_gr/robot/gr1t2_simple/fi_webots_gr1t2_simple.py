import os
import numpy
import torch

from robot_rcs_gr.rl.rl_actor_critic_mlp import ActorCriticMLP
from robot_rcs_gr.webots.webots_robot import WebotsRobot


def quat_rotate_inverse(q, v):
    shape = q.shape
    q_w = q[:, -1]
    q_vec = q[:, :3]
    a = v * (2.0 * q_w ** 2 - 1.0).unsqueeze(-1)
    b = torch.cross(q_vec, v, dim=-1) * q_w.unsqueeze(-1) * 2.0
    c = q_vec * torch.bmm(q_vec.view(shape[0], 1, 3), v.view(shape[0], 3, 1)).squeeze(-1) * 2.0
    return a - b + c


class WebotsGR1T2Simple(WebotsRobot):
    def __init__(self, sim_dt):
        super().__init__(sim_dt=sim_dt)

        self.robot_name = "GR1T2Simple"

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

        # pd
        self.joint_default_position = numpy.array([
            0.0, 0.0, -0.2618, 0.5236, -0.2618,  # left leg (5)
            0.0, 0.0, -0.2618, 0.5236, -0.2618,  # right leg (5)
        ])
        self.joint_pd_control_target = self.joint_default_position
        self.joint_pd_control_output = self.joint_default_position

        self.joint_pd_control_kp = numpy.array([
            57, 43, 114, 114, 15.3,  # left leg(5)
            57, 43, 114, 114, 15.3,  # right leg(5)
        ])
        self.joint_pd_control_kd = numpy.array([
            5.7, 4.3, 11.4, 11.4, 1.5,  # left leg(5)
            5.7, 4.3, 11.4, 11.4, 1.5,  # right leg(5)
        ])
        self.joint_pd_control_max = numpy.array([
            60.0, 45.0, 130.0, 130.0, 16.0,  # left leg(5)
            60.0, 45.0, 130.0, 130.0, 16.0,  # right leg(5)
        ])
        self.joint_pd_control_min = numpy.array([
            -60.0, -45.0, -130.0, -130.0, -16.0,  # left leg(5)
            -60.0, -45.0, -130.0, -130.0, -16.0,  # right leg(5)
        ])

        # --------------------------------------------------------------------------------
        # rl_walk algorithm
        self.decimation_count = 0
        self.decimation = 20

        # nerual network
        try:
            self.model_file_path = os.path.join(
                os.path.dirname(os.path.abspath(__file__)),
                "walk_model.pt"
            )

            model = torch.load(self.model_file_path, map_location=torch.device("cpu"))
            model_actor_dict = model["model_state_dict"]

            self.variable_nn_actor = \
                ActorCriticMLP(num_actor_obs=39,
                               num_critic_obs=168,
                               num_actions=self.num_of_joints,
                               actor_hidden_dims=[512, 256, 128],
                               critic_hidden_dims=[512, 256, 128])

            self.variable_nn_actor.load_state_dict(model_actor_dict)

        except Exception as e:
            print(e)

        self.variable_nn_actor_output = torch.zeros(1, self.num_of_joints, dtype=torch.float32)

        # actor clip
        self.variable_nn_actor_output_clip_max = torch.tensor([
            0.79, 0.7, 0.7, 1.92, 0.52,  # left leg (5), no ankle roll
            0.09, 0.7, 0.7, 1.92, 0.52,  # left leg (5), no ankle roll
        ])
        self.variable_nn_actor_output_clip_min = torch.tensor([
            -0.09, -0.7, -1.75, -0.09, -1.05,  # left leg (5), no ankle roll
            -0.79, -0.7, -1.75, -0.09, -1.05,  # left leg (5), no ankle roll
        ])
        self.variable_nn_actor_output_clip_max = self.variable_nn_actor_output_clip_max + 60 / 180 * torch.pi / 3
        self.variable_nn_actor_output_clip_min = self.variable_nn_actor_output_clip_min - 60 / 180 * torch.pi / 3

    def control_loop_algorithm(self):
        if self.decimation_count < 500:
            # stand control
            self.flag_joint_pd_torque_control = [
                False, False, False, False, False,  # left leg (5), no ankle roll
                False, False, False, False, False,  # right leg (5), no ankle roll
            ]
            self.flag_joint_position_control = [
                True, True, True, True, True,  # left leg (5), no ankle roll
                True, True, True, True, True,  # right leg (5), no ankle roll
            ]

            self.variable_nn_actor_output = torch.tensor([self.joint_measured_position_value], dtype=torch.float32) \
                                            - torch.tensor([self.joint_default_position], dtype=torch.float32)
            self.joint_pd_control_target = self.joint_default_position

        elif self.decimation_count >= 500 \
                and self.decimation_count % self.decimation == 0:
            # rl_walk control
            torch_commands = torch.tensor([[0, 0, 0]], dtype=torch.float32)
            torch_base_measured_quat_to_world = torch.tensor([self.imu_measured_quat_to_world], dtype=torch.float32)
            torch_base_measured_rpy_vel_to_world = torch.tensor([self.gyro_measured_rpy_vel_to_self], dtype=torch.float32)
            torch_joint_measured_position_value = torch.tensor([self.joint_measured_position_value], dtype=torch.float32)
            torch_joint_measured_velocity_value = torch.tensor([self.joint_measured_velocity_value], dtype=torch.float32)

            default_joint_position_tensor = torch.tensor([self.joint_default_position], dtype=torch.float32)
            gravity_vector = torch.tensor([[0.0, 0.0, -1.0]], dtype=torch.float32)

            # input
            actor_input_base_angular_velocity = torch_base_measured_rpy_vel_to_world
            actor_input_base_projected_gravity = quat_rotate_inverse(torch_base_measured_quat_to_world, gravity_vector)
            actor_input_command = torch_commands
            actor_input_offset_joint_position = torch_joint_measured_position_value - default_joint_position_tensor
            actor_input_measured_joint_velocity = torch_joint_measured_velocity_value
            actor_input_action = self.variable_nn_actor_output

            variable_nn_actor_input = torch.cat((
                actor_input_base_angular_velocity,
                actor_input_base_projected_gravity,
                actor_input_command,
                actor_input_offset_joint_position,
                actor_input_measured_joint_velocity,
                actor_input_action), dim=1)

            # actor output
            variable_nn_actor_output_tensor = \
                self.variable_nn_actor(variable_nn_actor_input)

            variable_nn_actor_output_raw = \
                variable_nn_actor_output_tensor.detach()
            variable_nn_actor_output_clip = \
                torch.clip(variable_nn_actor_output_raw,
                           min=self.variable_nn_actor_output_clip_min,
                           max=self.variable_nn_actor_output_clip_max)

            # store nn output
            self.variable_nn_actor_output = \
                variable_nn_actor_output_clip.clone()

            variable_nn_actor_output_pd_target = self.variable_nn_actor_output \
                                                 + default_joint_position_tensor

            # set to pd control target
            self.flag_joint_pd_torque_control = [
                True, True, True, True, True,  # left leg (5), no ankle roll
                True, True, True, True, True,  # right leg (5), no ankle roll
            ]
            self.flag_joint_position_control = [
                False, False, False, False, False,  # left leg (5), no ankle roll
                False, False, False, False, False,  # right leg (5), no ankle roll
            ]

            self.joint_pd_control_target = variable_nn_actor_output_pd_target.numpy()[0]

        else:
            pass

        # update count
        self.decimation_count += 1
