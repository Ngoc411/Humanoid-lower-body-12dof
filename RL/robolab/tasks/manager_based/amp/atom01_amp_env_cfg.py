# Copyright (c) 2022-2025, The Isaac Lab Project Developers.
# Copyright (c) 2025-2026, The RoboLab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice, this
#    list of conditions and the following disclaimer.
#
# 2. Redistributions in binary form must reproduce the above copyright notice,
#    this list of conditions and the following disclaimer in the documentation
#    and/or other materials provided with the distribution.
#
# 3. Neither the name of the copyright holder nor the names of its
#    contributors may be used to endorse or promote products derived from
#    this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

import os
from isaaclab.managers import RewardTermCfg as RewTerm
from isaaclab.managers import SceneEntityCfg
from isaaclab.utils import configclass
from isaaclab.envs import ViewerCfg

import robolab.tasks.manager_based.amp.mdp as mdp
from robolab.tasks.manager_based.amp.managers import MotionDataTermCfg
from robolab.tasks.manager_based.amp.amp_env_cfg import AmpEnvCfg, MotionDataCfg

import isaaclab.terrains as terrain_gen

##
# Pre-defined configs
##

from robolab.assets.robots.roboparty import ATOM01_CFG
from robolab.assets.robots.chaos import CHAOS_CFG
from robolab import ROBOLAB_ROOT_DIR

ISAACLAB_JOINT_ORDER= [
    # 'left_thigh_yaw_joint', 
    # 'right_thigh_yaw_joint', 
    # 'torso_joint', 
    # 'left_thigh_roll_joint', 
    # 'right_thigh_roll_joint', 
    # 'left_arm_pitch_joint', 
    # 'right_arm_pitch_joint', 
    # 'left_thigh_pitch_joint', 
    # 'right_thigh_pitch_joint', 
    # 'left_arm_roll_joint', 
    # 'right_arm_roll_joint', 
    # 'left_knee_joint', 
    # 'right_knee_joint', 
    # 'left_arm_yaw_joint', 
    # 'right_arm_yaw_joint', 
    # 'left_ankle_pitch_joint', 
    # 'right_ankle_pitch_joint', 
    # 'left_elbow_pitch_joint', 
    # 'right_elbow_pitch_joint', 
    # 'left_ankle_roll_joint', 
    # 'right_ankle_roll_joint', 
    # 'left_elbow_yaw_joint', 
    # 'right_elbow_yaw_joint'
    'left_hip_pitch_joint',
    'right_hip_pitch_joint',
    'left_hip_roll_joint',
    'right_hip_roll_joint',
    'left_hip_yaw_joint',
    'right_hip_yaw_joint',
    'left_knee_joint',
    'right_knee_joint',
    'left_ankle_pitch_joint',
    'right_ankle_pitch_joint',
    'left_ankle_roll_joint',
    'right_ankle_roll_joint'
]

DATASET_JOINT_ORDER = [
    # 'left_thigh_yaw_joint',
    # 'left_thigh_roll_joint',
    # 'left_thigh_pitch_joint',
    # 'left_knee_joint',
    # 'left_ankle_pitch_joint',
    # 'left_ankle_roll_joint',
    # 'right_thigh_yaw_joint',
    # 'right_thigh_roll_joint',
    # 'right_thigh_pitch_joint',
    # 'right_knee_joint',
    # 'right_ankle_pitch_joint',
    # 'right_ankle_roll_joint',
    # 'torso_joint',
    # 'left_arm_pitch_joint',
    # 'left_arm_roll_joint',
    # 'left_arm_yaw_joint',
    # 'left_elbow_pitch_joint',
    # 'left_elbow_yaw_joint',
    # 'right_arm_pitch_joint',
    # 'right_arm_roll_joint',
    # 'right_arm_yaw_joint',
    # 'right_elbow_pitch_joint',
    # 'right_elbow_yaw_joint',

    'left_hip_pitch_joint',
    'right_hip_pitch_joint',
    'left_hip_roll_joint',
    'right_hip_roll_joint',
    'left_hip_yaw_joint',
    'right_hip_yaw_joint',
    'left_knee_joint',
    'right_knee_joint',
    'left_ankle_pitch_joint',
    'right_ankle_pitch_joint',
    'left_ankle_roll_joint',
    'right_ankle_roll_joint'

    # 0:'left_hip_pitch_joint',
    # 1: 'right_hip_pitch_joint',
    # 2: 'left_hip_roll_joint',
    # 3: 'right_hip_roll_joint',
    # 4: 'left_hip_yaw_joint',
    # 5: 'right_hip_yaw_joint',
    # 6: 'left_knee_joint',
    # 7: 'right_knee_joint',
    # 8: 'left_ankle_pitch_joint',
    # 9: 'right_ankle_pitch_joint',
    # 10: 'left_ankle_roll_joint',
    # 11: 'right_ankle_roll_joint'
]

# The order must align with the retarget config file scripts/tools/retarget/config/g1_29dof.yaml
KEY_BODY_NAMES = [
    "left_ankle_roll_link", 
    "right_ankle_roll_link",
    "left_elbow_yaw_link",
    "right_elbow_yaw_link",
    "left_arm_roll_link",
    "right_arm_roll_link",
] # if changed here and symmetry is enabled, you might need to update amp.mdp.symmetry

ANIMATION_TERM_NAME = "animation"
AMP_NUM_STEPS = 3

@configclass
class Atom01AmpRewards():
    """Reward terms for the MDP."""

    # -- Task
    track_lin_vel_xy_exp = RewTerm(
        func=mdp.track_lin_vel_xy_exp,
        weight=0,
        params={"command_name": "base_velocity", "std": 0.5},
    )
    track_ang_vel_z_exp = RewTerm(
        func=mdp.track_ang_vel_z_exp, weight=0, params={"command_name": "base_velocity", "std": 0.5}
    )
    
    # -- Alive
    alive = RewTerm(func=mdp.is_alive, weight=0)
    
    # -- Base Link
    lin_vel_z_l2 = RewTerm(func=mdp.lin_vel_z_l2, weight=0)
    ang_vel_xy_l2 = RewTerm(func=mdp.ang_vel_xy_l2, weight=0)
    flat_orientation_l2 = RewTerm(func=mdp.flat_orientation_l2, weight=0)

    # -- Joint
    joint_vel_l2 = RewTerm(func=mdp.joint_vel_l2, weight=0)
    joint_acc_l2 = RewTerm(func=mdp.joint_acc_l2, weight=0)
    action_rate_l2 = RewTerm(func=mdp.action_rate_l2, weight=0)
    smoothness_1 = RewTerm(func=mdp.smoothness_1, weight=0)
    joint_pos_limits = RewTerm(func=mdp.joint_pos_limits, weight=0)
    joint_energy = RewTerm(func=mdp.joint_energy, weight=0)
    joint_regularization = RewTerm(func=mdp.joint_deviation_l1, weight=0)
    joint_torques_l2 = RewTerm(
        func=mdp.joint_torques_l2,
        weight=0.0,
    )

    low_speed_sway_penalty = RewTerm(
        func=mdp.low_speed_sway_penalty,
        weight=0.0,
        params={
            "command_name": "base_velocity",
            "command_threshold": 0.1,
        },
    )

    stand_still_joint_deviation_l1 = RewTerm(
        func=mdp.stand_still_joint_deviation_l1,
        weight=0.0,
        params={
            "command_name": "base_velocity",
            "command_threshold": 0.06,
        }
    )
        
    # -- Feet
    feet_slide = RewTerm(
        func=mdp.feet_slide,
        weight=0,
        params={
            "sensor_cfg": SceneEntityCfg("contact_forces", body_names=".*_ankle_roll"),
            "asset_cfg": SceneEntityCfg("robot", body_names=".*_ankle_roll"),
        },
    )
    
    feet_stumble = RewTerm(
        func=mdp.feet_stumble,
        weight=0.0,
        params={
            "sensor_cfg": SceneEntityCfg("contact_forces", body_names=".*_ankle_roll"),
        },
    )
    
    feet_air_time_positive_biped = RewTerm(
        func=mdp.feet_air_time_positive_biped,
        weight=0.0,
        params={
            "command_name": "base_velocity",
            "sensor_cfg": SceneEntityCfg("contact_forces", body_names=".*_ankle_roll"),
            "asset_cfg":  SceneEntityCfg("robot",          body_names=".*_ankle_roll"),
            "threshold": 0.4,
            "vel_cmd_threshold": 0.15,
            "penalty_on": False,
            "com_to_sole": 0.0375,
            "min_clearance": 0.02,
        },
    )
    
    sound_suppression = RewTerm(
        func=mdp.sound_suppression_acc_per_foot,
        weight=0,
        params={
            "sensor_cfg": SceneEntityCfg(
                "contact_forces",
                body_names=".*_ankle_roll",
            ),
        },
    )

    # -- other
    undesired_contacts = RewTerm(
        func=mdp.undesired_contacts,
        weight=-1,
        params={
            "threshold": 1,
            "sensor_cfg": SceneEntityCfg("contact_forces", body_names=["(?!.*ankle.*).*"]),
        },
    )


@configclass
class Atom01AmpEnvCfg(AmpEnvCfg):
    rewards: Atom01AmpRewards = Atom01AmpRewards()

    def __post_init__(self):
        # post init of parent
        super().__post_init__()
        
        # ------------------------------------------------------
        # Scene
        # ------------------------------------------------------
        # self.scene.robot = ATOM01_CFG.replace(prim_path="{ENV_REGEX_NS}/Robot")
        self.scene.robot = CHAOS_CFG.replace(prim_path="{ENV_REGEX_NS}/Robot")
        
        # plane terrain
        # self.scene.terrain.terrain_type = "plane"
        # self.scene.terrain.terrain_generator = None

        # ------------------------------------------------------
        # motion data
        # ------------------------------------------------------
        # self.motion_data.motion_dataset.motion_data_dir = os.path.join(
        #     ROBOLAB_ROOT_DIR, "data", "motions", "atom01_lab"
        # )
        self.motion_data.motion_dataset.motion_data_dir = os.path.join(ROBOLAB_ROOT_DIR, "data", "motions")
        self.motion_data.motion_dataset.motion_data_weights={
            
            # # # # 4
            # # '02_02': 1,
            # # '16_34': 1,
            
            # "127_03": 1,
            # "127_04": 1,
            # "127_06": 1,
            # # "143_03": 1,
            
            # # # # standstill 1
            # "A1-_Stand_stageii": 1,
            
            
            # # # # male2 walk 8
            # "B4_-_Stand_to_Walk_backwards_stageii":1,
            # "B9_-__Walk_turn_left_90_stageii":1,
            # "B10_-__Walk_turn_left_45_stageii":1,
            # "B13_-__Walk_turn_right_90_stageii":1,
            # "B14_-__Walk_turn_right_45_t2_stageii":1,
            # "B15_-__Walk_turn_around_stageii":1,
            # # "B15_-__Walk_turn_around_stageii_turn":1,
            # # "B15_-__Walk_turn_around_stageii_walk":1,
            # "B22_-__side_step_left_stageii":1,
            # "B23_-__side_step_right_stageii":1,
            
            # # # male2 run 8
            # # "C1_-_stand_to_run_stageii": 1,
            # # "C3_-_run_stageii": 1,
            # # "C4_-_run_to_walk_a_stageii": 1,
            # # "C4_-_run_to_walk_stageii":1,
            # # "C5_-_walk_to_run_stageii":1,
            # "C12_-_run_turn_left_45_stageii":1,
            # # "C15_-_run_turn_right_45_stageii":1,
            # "C17_-_run_change_direction_stageii":1,
            

            # "xuanzhuan":1,
            # # "xuanzhuan_l":1,
            # # "xuanzhuan_r":1,

            # "chaos_walk_forward": 1,
            # "chaos_walk_50": 1,
            # "chaos_walk_100": 1,
            "chaos_stand": 1,
            # "walk1_backward_retargeted_ik_01": 1,
            # "walk1_backward_retargeted_ik_02": 1,
            # "walk1_backward_retargeted_ik_03": 1,
            "walk1_forward_retargeted_ik_01": 1,
            "walk1_forward_retargeted_ik_02": 1,
            "walk1_forward_retargeted_ik_03": 1,
            # "walk1_forward_retargeted_ik_09": 1,
            # "walk1_forward_retargeted_ik_10": 1,
            # "walk1_forward_retargeted_ik_12": 1,
            # "walk1_forward_retargeted_ik_14": 1,
            # "walk1_forward_retargeted_ik_15": 1,
            # "walk1_forward_retargeted_ik_16": 1,

            # "walk1_left_retargeted_ik_01": 1,
            # "walk1_left_retargeted_ik_02": 1,
            # "walk1_right_retargeted_ik_01": 1,
            # "walk1_retargeted_ik": 1,
            # "chaos_walk_backward": 1,
            # "chaos_side_walk_left": 1,
            # "chaos_side_walk_right": 1,


        }
        
        # ------------------------------------------------------
        # animation
        # ------------------------------------------------------
        self.animation.animation.num_steps_to_use = AMP_NUM_STEPS

        # ------------------------------------------------------
        # Observations
        # ------------------------------------------------------
                
        # discriminator observations
        
        # self.observations.disc.key_body_pos_b.params = {
        #     "asset_cfg": SceneEntityCfg(
        #         name="robot", 
        #         body_names=KEY_BODY_NAMES, 
        #         preserve_order=True
        #     )
        # }
        self.observations.disc.history_length = AMP_NUM_STEPS
        # self.observations.disc_demo.history_length = 1

        # discriminator demonstration observations
        
        self.observations.disc_demo.ref_root_local_rot_tan_norm.params["animation"] = ANIMATION_TERM_NAME
        self.observations.disc_demo.ref_root_lin_vel_b.params["animation"] = ANIMATION_TERM_NAME
        self.observations.disc_demo.ref_root_ang_vel_b.params["animation"] = ANIMATION_TERM_NAME
        self.observations.disc_demo.ref_joint_pos.params["animation"] = ANIMATION_TERM_NAME
        self.observations.disc_demo.ref_joint_vel.params["animation"] = ANIMATION_TERM_NAME
        # should use motion instead

        # self.observations.disc_demo.ref_key_body_pos_b.params["animation"] = ANIMATION_TERM_NAME
     
        # ------------------------------------------------------
        # Events
        # ------------------------------------------------------

        # ------------------------------------------------------
        # Rewards
        # ------------------------------------------------------
        # task
        self.rewards.track_lin_vel_xy_exp.weight = 1.75
        self.rewards.track_ang_vel_z_exp.weight = 1.75
        self.rewards.alive.weight = 0.15
        
        # base
        # self.rewards.lin_vel_z_l2.weight = -0.1
        self.rewards.ang_vel_xy_l2.weight = -0.1
        self.rewards.flat_orientation_l2.weight = -1.0
        
        # joint
        self.rewards.joint_vel_l2.weight = -2e-4
        self.rewards.joint_acc_l2.weight = -2.5e-7
        self.rewards.action_rate_l2.weight = -0.01
        self.rewards.joint_pos_limits.weight = -1.0
        self.rewards.joint_energy.weight = -1e-4
        self.rewards.joint_torques_l2.weight = -1e-5
        self.rewards.joint_regularization.weight = -1e-3
        self.rewards.low_speed_sway_penalty.weight = -1.0e-2
        # self.rewards.stand_still_joint_deviation_l1.weight = -1.0e-1
        
        # feet
        self.rewards.feet_slide.weight = -0.1
        self.rewards.feet_stumble.weight = -0.1
        self.rewards.sound_suppression.weight = -5e-5
        self.rewards.feet_air_time_positive_biped.weight = 1.0


        self.rewards.undesired_contacts.weight = -1.0
        self.rewards.undesired_contacts.params["sensor_cfg"] = SceneEntityCfg(
            "contact_forces",
            body_names=["(?!.*ankle.*).*"],  # exclude ankle links
        )
        
        # ------------------------------------------------------
        # Commands
        # ------------------------------------------------------
        
        self.commands.base_velocity.ranges.lin_vel_x = (-0.8, 2.0)
        self.commands.base_velocity.ranges.lin_vel_y = (-0.8, 0.8)
        self.commands.base_velocity.ranges.ang_vel_z = (-0.8, 0.8)
                
        # ------------------------------------------------------
        # Curriculum
        # ------------------------------------------------------
        
        self.terminations.base_contact.params["sensor_cfg"].body_names = [
            ".*hip.*", "base_link",
            # "(?!.*ankle.*).*"
        ]

        # Disable contact sensor — CHAOS USD may not have PhysicsRigidBodyAPI on child links.
        # Remove this block once USD is regenerated with proper collision APIs.
        # self.scene.contact_forces = None
        # self.rewards.feet_slide = None
        # self.rewards.feet_stumble = None
        # self.rewards.sound_suppression = None
        # self.rewards.feet_air_time_positive_biped = None
        # self.rewards.undesired_contacts = None
        # self.terminations.base_contact = None

        if self.__class__.__name__ == "Atom01AmpEnvCfg":
            self.disable_zero_weight_rewards()
            
            
@configclass
class Atom01AmpEnvCfg_PLAY(Atom01AmpEnvCfg):
    def __post_init__(self):
        # post init of parent
        super().__post_init__()
        
        self.viewer = ViewerCfg(
            eye=(3.0, 3.0, 3.0),
            lookat=(0.0, 0.0, 0.5),
            origin_type="asset_root",
            env_index=0,
            asset_name="robot",
        )

        # make a smaller scene for play
        self.scene.num_envs = 1
        self.scene.env_spacing = 2.5
        self.episode_length_s = 40.0

        self.commands.base_velocity.ranges.lin_vel_x = (0.0, 0.0)
        self.commands.base_velocity.ranges.lin_vel_y = (0.0, 0.0)
        self.commands.base_velocity.ranges.ang_vel_z = (0.0, 0.0)

        # disable randomization for play
        self.observations.policy.enable_corruption = False
        # remove random pushing
        self.events.push_robot = None

        # rough + wave terrain for visual testing
        self.scene.terrain.terrain_type = "generator"
        self.scene.terrain.terrain_generator = terrain_gen.TerrainGeneratorCfg(
            size=(8.0, 8.0),
            border_width=2.0,        # 20→2: thu nhỏ viền phẳng, robot đi vào rough ngay
            num_rows=4,
            num_cols=4,              # 16 ô ghồ ghề thay vì 4
            horizontal_scale=0.05,
            vertical_scale=0.005,
            slope_threshold=0.75,
            sub_terrains={
                "flat": terrain_gen.MeshPlaneTerrainCfg(
                    proportion=0.4,
                ),
                # "random_rough": terrain_gen.HfRandomUniformTerrainCfg(
                #     proportion=0.3,
                #     noise_range=(0.02, 0.06),  # 3cm→6cm: ghồ ghề hơn
                #     noise_step=0.005,           # bước nhảy mịn hơn
                #     border_width=0.25,
                # ),
                # "wave": terrain_gen.HfWaveTerrainCfg(
                #     proportion=0.3,
                #     amplitude_range=(0.03, 0.08),  # sóng cao hơn
                #     num_waves=5,                    # 2→3 chu kỳ
                # ),
            },
        )