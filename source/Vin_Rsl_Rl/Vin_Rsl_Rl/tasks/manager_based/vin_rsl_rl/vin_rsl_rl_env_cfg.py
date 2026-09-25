# Copyright (c) 2022-2025, The Isaac Lab Project Developers (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

import math

from dataclasses import MISSING
# import torch

import isaaclab.sim as sim_utils
from isaaclab.assets import ArticulationCfg, AssetBaseCfg
from isaaclab.envs import ManagerBasedRLEnvCfg
from isaaclab.managers import CurriculumTermCfg as CurrTerm
from isaaclab.managers import EventTermCfg as EventTerm
from isaaclab.managers import ObservationGroupCfg as ObsGroup
from isaaclab.managers import ObservationTermCfg as ObsTerm
from isaaclab.managers import RewardTermCfg as RewTerm
from isaaclab.managers import SceneEntityCfg
from isaaclab.managers import TerminationTermCfg as DoneTerm
from isaaclab.scene import InteractiveSceneCfg
from isaaclab.utils import configclass
from isaaclab.utils.noise import AdditiveUniformNoiseCfg as Unoise
from isaaclab.terrains import TerrainImporterCfg
import isaaclab.terrains as terrain_gen
from isaaclab.utils.assets import ISAAC_NUCLEUS_DIR, ISAACLAB_NUCLEUS_DIR
from isaaclab.sensors import ContactSensorCfg


from . import mdp

##
# Pre-defined configs
##

from Vin_Rsl_Rl.assets.robots.chaos import CHAOS_CFG


##
# Chaos 12 DOF specific constants
##

ROOT_BODY = "base_link"

FOOT = SceneEntityCfg("robot", body_names=["left_ankle_roll", "right_ankle_roll"])
FOOT_OFFSET = (0.0, 0.0, -0.0385)

FOOT_LINK_NAMES = ("left_ankle_roll", "right_ankle_roll")

# Pose reward stds â€” only has leg joints. Hip roll/yaw stay tight to
# approximate Isaac's `joint_deviation_l1` on those joints (which keeps the
# legs aligned forward).
POSE_STD_STANDING: dict[str, float] = {".*": 0.05}
POSE_STD_WALKING: dict[str, float] = {
    # Lower body.
    r".*hip_pitch.*": 0.5,
    r".*hip_roll.*": 0.5,
    r".*hip_yaw.*": 0.1,
    r".*knee.*": 0.65, # was 0.5
    r".*ankle_pitch.*": 0.5,
    r".*ankle_roll.*": 0.1,
}
POSE_STD_RUNNING: dict[str, float] = {
    r".*hip_pitch.*": 1.0,
    r".*hip_roll.*": 1.0,
    r".*hip_yaw.*": 0.1,
    r".*knee.*": 1.0,
    r".*ankle_pitch.*": 1.0,
    r".*ankle_roll.*": 0.1,
}
GAIT_CYCLE_PERIOD = 0.9
OBS_HISTORY_LENGTH = 5


##
# Scene definition
##


@configclass
class VinRslRlSceneCfg(InteractiveSceneCfg):
    """Configuration for the Chaos velocity scene."""

    # ground terrain â€” curriculum: 10 levels Ã— 3 types (flat / rough / wave)
    terrain = TerrainImporterCfg(
        prim_path="/World/ground",
        terrain_type="generator",
        terrain_generator=terrain_gen.TerrainGeneratorCfg(
            size=(8.0, 8.0),
            border_width=1.0,
            num_rows=10,
            num_cols=10,
            horizontal_scale=0.1,
            vertical_scale=0.005,
            slope_threshold=0.75,
            curriculum=True,
            sub_terrains={
                "flat": terrain_gen.MeshPlaneTerrainCfg(proportion=0.5),
                "gravel": terrain_gen.HfRandomUniformTerrainCfg(
                    proportion=0.25,
                    noise_range=(0.0, 0.03),
                    noise_step=0.005,
                    border_width=0.25,
                ),
                "grid": terrain_gen.MeshRandomGridTerrainCfg(
                    proportion=0.25,
                    grid_width=0.45,
                    grid_height_range=(0.01, 0.05),
                    platform_width=2.0,
                ),
            },
        ),
        max_init_terrain_level=5,
        # terrain_type="generator",
        # terrain_generator=terrain_gen.TerrainGeneratorCfg(
        #     size=(16.0, 16.0),
        #     border_width=20.0,
        #     num_rows=10,
        #     num_cols=3,
        #     horizontal_scale=0.1,
        #     vertical_scale=0.005,
        #     slope_threshold=0.75,
        #     curriculum=True,
        #     sub_terrains={
        #         "flat": terrain_gen.MeshPlaneTerrainCfg(
        #             proportion=0.6,
        #         ),
        #         "random_rough": terrain_gen.HfRandomUniformTerrainCfg(
        #             proportion=0.2,
        #             noise_range=(0.0, 0.06),
        #             noise_step=0.01,
        #             border_width=0.25,
        #         ),
        #         "wave": terrain_gen.HfWaveTerrainCfg(
        #             proportion=0.2,
        #             amplitude_range=(0.0, 0.15),
        #             num_waves=6,
        #         ),
        #     },
        # ),
        # max_init_terrain_level=0,
        collision_group=-1,
        physics_material=sim_utils.RigidBodyMaterialCfg(
            friction_combine_mode="multiply",
            restitution_combine_mode="multiply",
            static_friction=1.0,
            dynamic_friction=1.0,
        ),
        visual_material=sim_utils.MdlFileCfg(
            mdl_path=f"{ISAACLAB_NUCLEUS_DIR}/Materials/TilesMarbleSpiderWhiteBrickBondHoned/TilesMarbleSpiderWhiteBrickBondHoned.mdl",
            project_uvw=True,
            texture_scale=(0.25, 0.25),
        ),
        debug_vis=False,
    )

    # robots
    robot: ArticulationCfg = MISSING
    # robot animation (for reference)
    robot_anim: ArticulationCfg = None
    # sensors
    feet_ground_contact = ContactSensorCfg(
        prim_path="{ENV_REGEX_NS}/Robot/assem_symplified_7/.*ankle_roll",
        history_length=4,
        track_air_time=True,
    )
    # self_collision = ContactSensorCfg(
    #     prim_path="{ENV_REGEX_NS}/Robot/assem_symplified_7/.*",
    #     history_length=4,
    #     track_air_time=False,
    # )
    
    # lights
    sky_light = AssetBaseCfg(
        prim_path="/World/skyLight",
        spawn=sim_utils.DomeLightCfg(
            intensity=750.0,
            texture_file=f"{ISAAC_NUCLEUS_DIR}/Materials/Textures/Skies/PolyHaven/kloofendal_43d_clear_puresky_4k.hdr",
        ),
    )

@configclass
class VinRslRlFlatSceneCfg(VinRslRlSceneCfg):
    """Lighter Chaos scene with a plain ground plane."""

    terrain = TerrainImporterCfg(
        prim_path="/World/ground",
        terrain_type="plane",
        collision_group=-1,
        physics_material=sim_utils.RigidBodyMaterialCfg(
            friction_combine_mode="multiply",
            restitution_combine_mode="multiply",
            static_friction=1.0,
            dynamic_friction=1.0,
        ),
        visual_material=sim_utils.MdlFileCfg(
            mdl_path=f"{ISAACLAB_NUCLEUS_DIR}/Materials/TilesMarbleSpiderWhiteBrickBondHoned/TilesMarbleSpiderWhiteBrickBondHoned.mdl",
            project_uvw=True,
            texture_scale=(0.25, 0.25),
        ),
        debug_vis=False,
    )

##
# MDP settings
##


@configclass
class ActionsCfg:
    """Action specifications for the MDP."""

    joint_pos = mdp.JointPositionActionCfg(asset_name="robot", joint_names=[".*"], scale=0.25, use_default_offset=True)


# Commands
@configclass
class CommandsCfg:
    twist = mdp.UniformVelocityCommandCfg(
        asset_name="robot",
        resampling_time_range=(3.0, 8.0),
        rel_standing_envs=0.1,
        heading_command=False,
        heading_control_stiffness=0.5,
        debug_vis=True,
        ranges=mdp.UniformVelocityCommandCfg.Ranges(
            lin_vel_x=(-1.0, 2.0),
            lin_vel_y=(-0.5, 0.5),
            ang_vel_z=(-1.0, 1.0),
            heading=None,
        ),
    )


# Observations

@configclass
class ObservationsCfg:
    """Observation specifications for the MDP."""

    @configclass
    class ActorCfg(ObsGroup):
        """Observations for policy group."""

        base_ang_vel = ObsTerm(
            func=mdp.base_ang_vel,
            scale=0.25,
            noise=Unoise(n_min=-0.75, n_max=0.75),
            history_length=OBS_HISTORY_LENGTH,
            flatten_history_dim=True,
        )
        projected_gravity = ObsTerm(
            func=mdp.projected_gravity,
            noise=Unoise(n_min=-0.15, n_max=0.15),
            history_length=OBS_HISTORY_LENGTH,
            flatten_history_dim=True,
        )
        velocity_commands = ObsTerm(
            func=mdp.generated_commands,
            params={"command_name": "twist"},
            history_length=OBS_HISTORY_LENGTH,
            flatten_history_dim=True,
        )

        joint_pos_rel = ObsTerm(
            func=mdp.joint_pos_rel,
            noise=Unoise(n_min=-0.12, n_max=0.12),
            history_length=OBS_HISTORY_LENGTH,
            flatten_history_dim=True,
        )  # approximate 5.7deg
        joint_vel_rel = ObsTerm(
            func=mdp.joint_vel_rel,
            scale=0.05,
            noise=Unoise(n_min=-1.75, n_max=1.75),
            history_length=OBS_HISTORY_LENGTH,
            flatten_history_dim=True,
        )  # aproximate 16.71RPM

        last_action = ObsTerm(
            func=mdp.last_action,
            history_length=OBS_HISTORY_LENGTH,
            flatten_history_dim=True,
        )

        gait_phase = ObsTerm(
            func=mdp.phase,
            params={"period": GAIT_CYCLE_PERIOD, "command_name": "twist"},
            history_length=OBS_HISTORY_LENGTH,
            flatten_history_dim=True,
        )

        def __post_init__(self) -> None:
            self.enable_corruption = True
            self.concatenate_terms = True

    policy: ActorCfg = ActorCfg()

    @configclass
    class CriticCfg(ObsGroup):
        base_ang_vel = ObsTerm(
            func=mdp.base_ang_vel,
            history_length=OBS_HISTORY_LENGTH,
            flatten_history_dim=True,
        )
        projected_gravity = ObsTerm(
            func=mdp.projected_gravity,
            history_length=OBS_HISTORY_LENGTH,
            flatten_history_dim=True,
        )
        velocity_commands = ObsTerm(
            func=mdp.generated_commands,
            params={"command_name": "twist"},
            history_length=OBS_HISTORY_LENGTH,
            flatten_history_dim=True,
        )

        joint_pos_rel = ObsTerm(
            func=mdp.joint_pos_rel,
            history_length=OBS_HISTORY_LENGTH,
            flatten_history_dim=True,
        )  # approximate 5.7deg
        joint_vel_rel = ObsTerm(
            func=mdp.joint_vel_rel,
            history_length=OBS_HISTORY_LENGTH,
            flatten_history_dim=True,
        )  # aproximate 16.71RPM

        last_action = ObsTerm(
            func=mdp.last_action,
            history_length=OBS_HISTORY_LENGTH,
            flatten_history_dim=True,
        )

        gait_phase = ObsTerm(
            func=mdp.phase,
            params={"period": GAIT_CYCLE_PERIOD, "command_name": "twist"},
            history_length=OBS_HISTORY_LENGTH,
            flatten_history_dim=True,
        )
        true_base_velocity = ObsTerm(
            func=mdp.base_lin_vel,
            history_length=OBS_HISTORY_LENGTH,
            flatten_history_dim=True,
        )
        foot_height = ObsTerm(
            func=mdp.foot_height,
            params={"asset_cfg": FOOT, "foot_offset": FOOT_OFFSET},
            history_length=OBS_HISTORY_LENGTH,
            flatten_history_dim=True,
        )
        foot_air_time = ObsTerm(
            func=mdp.foot_air_time,
            params={"sensor_name": "feet_ground_contact"},
            history_length=OBS_HISTORY_LENGTH,
            flatten_history_dim=True,
        )
        foot_contact = ObsTerm(
            func=mdp.foot_contact,
            params={"sensor_name": "feet_ground_contact"},
            history_length=OBS_HISTORY_LENGTH,
            flatten_history_dim=True,
        )
        foot_contact_forces = ObsTerm(
            func=mdp.foot_contact_forces,
            params={"sensor_name": "feet_ground_contact"},
            history_length=OBS_HISTORY_LENGTH,
            flatten_history_dim=True,
        )
        knee_reference = ObsTerm(
            func=mdp.get_desired_knee_angle,
            params={
                "period": GAIT_CYCLE_PERIOD,
                "offset": [0.0, 0.5],
                "command_name": "twist",
            },
            history_length=OBS_HISTORY_LENGTH,
            flatten_history_dim=True,
        )

        def __post_init__(self) -> None:
            self.enable_corruption = False
            self.concatenate_terms = True

    critic: CriticCfg = CriticCfg()
# Events

@configclass
class EventCfg:
    """Configuration for events."""

    reset_base = EventTerm(
        func=mdp.reset_root_state_uniform,
        mode="reset",
        params={
            "pose_range": {
                "x": (-0.5, 0.5),
                "y": (-0.5, 0.5),
                "z": (0.0, 0.0),
                "yaw": (-3.14, 3.14),
            },
            "velocity_range": {},
        },
    )

    reset_robot_joints = EventTerm(
        func=mdp.reset_joints_by_offset,
        mode="reset",
        params={
            "position_range": (-0.1, 0.1),
            "velocity_range": (-0.5, 0.5),
            "asset_cfg": SceneEntityCfg("robot", joint_names=(".*",)),
        },
    )

    push_robot = EventTerm(
        func=mdp.push_by_setting_velocity,
        mode="interval",
        interval_range_s=(5.0, 6.0),
        params={
            "velocity_range": {
                "x": (-0.5, 0.5),
                "y": (-0.5, 0.5),
                "z": (-0.4, 0.4),
                "roll": (-0.52, 0.52),
                "pitch": (-0.52, 0.52),
                "yaw": (-0.78, 0.78),
            },
        },
    )

    foot_friction = EventTerm(
        func=mdp.randomize_rigid_body_material,
        mode="startup",
        params={
            "asset_cfg": SceneEntityCfg("robot", body_names=FOOT_LINK_NAMES),
            "static_friction_range": (0.3, 1.6),
            "dynamic_friction_range": (0.3, 1.6),
            "restitution_range": (0.0, 0.0),
            "num_buckets": 64,
            "make_consistent": True,
        },
    )

    base_com = EventTerm(
        func=mdp.randomize_rigid_body_com,
        mode="startup",
        params={
            "asset_cfg": SceneEntityCfg("robot", body_names=(ROOT_BODY,)),
            "com_range": {
                "x": (-0.05, 0.05),
                "y": (-0.05, 0.05),
                "z": (-0.05, 0.05),
            },
        },
    )

    body_com = EventTerm(
        func=mdp.randomize_rigid_body_com,
        mode="startup",
        params={
            "asset_cfg": SceneEntityCfg("robot", body_names=(f"^(?!{ROOT_BODY}$).*",)),
            "com_range": {
                "x": (-0.03, 0.03),
                "y": (-0.03, 0.03),
                "z": (-0.03, 0.03),
            },
        },
    )

    base_mass = EventTerm(
        func=mdp.randomize_rigid_body_mass,
        mode="startup",
        params={
            "asset_cfg": SceneEntityCfg("robot", body_names=(ROOT_BODY,)),
            "operation": "add",
            "mass_distribution_params": (-1.0, 1.0),
        },
    )

    body_mass = EventTerm(
        func=mdp.randomize_rigid_body_mass,
        mode="startup",
        params={
            "asset_cfg": SceneEntityCfg("robot", body_names=(f"^(?!{ROOT_BODY}$).*",)),
            "operation": "scale",
            "mass_distribution_params": (0.9, 1.1),
        },
    )

    randomize_actuator_gains = EventTerm(
        func=mdp.randomize_actuator_gains,
        mode="startup",
        params={
            "asset_cfg": SceneEntityCfg("robot", joint_names=(".*",)),
            "operation": "scale",
            "stiffness_distribution_params": (0.7, 1.1),
            "damping_distribution_params": (0.7, 1.1),
        },
    )

    randomize_joint_limit = EventTerm(
        func=mdp.randomize_joint_parameters,
        mode="startup",
        params={
            "asset_cfg": SceneEntityCfg("robot", joint_names=(".*",)),
            "operation": "scale",
            "lower_limit_distribution_params": (0.94, 1.06),
            "upper_limit_distribution_params": (0.94, 1.06),
        },
    )

    randomize_joint_frictionloss = EventTerm(
        func=mdp.randomize_joint_parameters,
        mode="startup",
        params={
            "asset_cfg": SceneEntityCfg("robot", joint_names=(".*",)),
            "operation": "scale",
            "friction_distribution_params": (0.1, 1.3),
        },
    )

    randomize_joint_armature = EventTerm(
        func=mdp.randomize_joint_parameters,
        mode="startup",
        params={
            "asset_cfg": SceneEntityCfg("robot", joint_names=(".*",)),
            "operation": "scale",
            "armature_distribution_params": (0.5, 2.0),
        },
    )

    randomize_joint_stiffness = EventTerm(
        func=mdp.randomize_actuator_gains,
        mode="startup",
        params={
            "asset_cfg": SceneEntityCfg("robot", joint_names=(".*",)),
            "operation": "scale",
            "stiffness_distribution_params": (0.7, 1.3),
        },
    )

    randomize_joint_damping = EventTerm(
        func=mdp.randomize_actuator_gains,
        mode="startup",
        params={
            "asset_cfg": SceneEntityCfg("robot", joint_names=(".*",)),
            "operation": "scale",
            "damping_distribution_params": (0.7, 1.3),
        },
    )

# Rewards

@configclass
class RewardsCfg:
    """Reward terms for the MDP."""

    track_linear_x = RewTerm(
        func=mdp.track_linear_x,
        weight=2.0,
        params={
            "command_name": "twist",
            "std_list": [0.5, 0.25],
            "vel_list": [0.0, 0.1, 2.0],
            "add_penalize": True,
            "penalize_scale": 1.0,
        },
    )
    track_linear_y = RewTerm(
        func=mdp.track_linear_y,
        weight=2.0,
        params={
            "command_name": "twist",
            "std_list": [0.5, 0.25],
            "vel_list": [0.0, 0.1, 2.0],
            "add_penalize": True,
            "penalize_scale": 1.0,
        },
    )
    track_angular_z = RewTerm(
        func=mdp.track_angular_z,
        weight=2.0,
        params={
            "command_name": "twist",
            "std_list": [0.5, 0.25],
            "vel_list": [0.0, 0.1, 2.0],
            "add_penalize": True,
            "penalize_scale": 1.0,
        },
    )
    linear_vel_z = RewTerm(func=mdp.lin_vel_z_l2, weight=-2.0)
    # Weight bumped -0.05 -> -0.15 to absorb the removed `body_ang_vel` term (-0.1),
    # which penalized the same base xy angular velocity (see note near is_terminated).
    angular_vel_xy = RewTerm(func=mdp.ang_vel_xy_l2, weight=-0.15)
    flat_orientation = RewTerm(func=mdp.flat_orientation_l2, weight=-4.0) # was -5.0
    # NOTE: `link_orientation` on base_link was removed — it duplicated
    # `flat_orientation` almost exactly (both target ROOT_BODY == base_link and
    # penalize projected-gravity xy of the same body). Re-add only if you target a
    # separate torso/waist link that is physically distinct from the root.
    pelvis_roll_pitch = RewTerm(
        func=mdp.body_roll_pitch_penalty,
        weight=-1.0,
        params={
            "asset_cfg": SceneEntityCfg("robot", body_names=(ROOT_BODY,)),
            "weight_list": [100.0],
            "velocity_list": [0.0, 2.0],
            "command_name": "twist",
        },
    )
    pose = RewTerm(
        func=mdp.variable_posture,
        weight=1.0,
        params={
            "asset_cfg": SceneEntityCfg("robot", joint_names=(".*",)),
            "command_name": "twist",
            "std_standing": POSE_STD_STANDING,
            "std_walking": POSE_STD_WALKING,
            "std_running": POSE_STD_RUNNING,
            "walking_threshold": 0.1,
            "running_threshold": 1.5,
        },
    )
    feet_air_time_biped = RewTerm(
        func=mdp.feet_air_time_positive_biped,
        weight=1.75, # was 1.5, was 1.7
        params={
            "command_name": "twist",
            "sensor_name": "feet_ground_contact",
            "threshold": 0.52,
        },
    )
    foot_clearance = RewTerm( 
        func=mdp.feet_clearance,
        weight=-1.75, # was -1.0, was -1.2, was -1.5
        params={
            "target_height": 0.1,
            "command_name": "twist",
            "command_threshold": 0.1,
            "asset_cfg": FOOT,
            "foot_offset": FOOT_OFFSET,
        },
    )
    foot_slip = RewTerm(
        func=mdp.feet_slip,
        weight=-0.25,
        params={
            "sensor_name": "feet_ground_contact",
            "command_name": "twist",
            "command_threshold": 0.1,
            "asset_cfg": FOOT,
            "foot_offset": FOOT_OFFSET,
        },
    )
    feet_close_xy = RewTerm(
        func=mdp.feet_close_xy_gauss,
        weight=1.0,
        params={
            "threshold": 0.18,
            "asset_cfg": SceneEntityCfg("robot", body_names=FOOT_LINK_NAMES),
            "std": math.sqrt(0.05),
            "foot_offset": FOOT_OFFSET,
        },
    )
    action_rate_l2 = RewTerm(func=mdp.action_rate_l2, weight=-0.75) # was -0.5, was -0.75, was -1.0, was -1.5
    joint_acc_l2 = RewTerm(func=mdp.joint_acc_l2, weight=-1.0e-5) # was -2.5e-7, was -5.0e-7, was -2.5e-6
    dof_torques_l2 = RewTerm(
        func=mdp.joint_torques_l2,
        weight=-1.0e-7,
        params={
            "asset_cfg": SceneEntityCfg(
                "robot",
                joint_names=(".*_hip_.*", ".*_knee_.*", ".*_ankle_.*"),
            )
        },
    )
    # NOTE: `body_ang_vel` (body_angular_velocity_penalty on base_link) was removed —
    # it duplicated `angular_vel_xy` (same base xy angular velocity; world vs body
    # frame is ~identical for a near-upright base). Its weight was folded into
    # `angular_vel_xy` above (-0.05 -> -0.15).
    is_terminated = RewTerm(func=mdp.is_terminated, weight=-50.0)
    joint_pos_limits = RewTerm(func=mdp.joint_pos_limits, weight=-2.5) # was -5.0
    stand_still = RewTerm(
        func=mdp.stand_still,
        weight=-1.0,
        params={
            "command_name": "twist",
            "command_threshold": 0.1,
            "asset_cfg": SceneEntityCfg("robot", joint_names=(".*",)),
        },
    )
    dont_wait = RewTerm(
        func=mdp.dont_wait,
        weight=-0.5,
        params={
            "command_name": "twist",
            "cmd_threshold": 0.1,
        },
    )
    knee_motion = RewTerm(
        func=mdp.knee_joint_motion,
        weight=0.75, # was 0.5
        params={
            "reward_limit": 1.0,
            "command_name": "twist",
            "command_threshold": 0.1,
            "asset_cfg": SceneEntityCfg("robot", joint_names=(".*knee.*",)),
        },
    )
# Terminations

@configclass
class TerminationsCfg:
    """Termination terms for the MDP."""

    time_out = DoneTerm(func=mdp.time_out, time_out=True)
    fell_over = DoneTerm(
        func=mdp.bad_orientation,
        params={"limit_angle": math.radians(70.0)},
    )
    # root_height = DoneTerm(
    #     func=mdp.root_height_below_minimum,
    #     params={"minimum_height": 0.3},
    # )
    nan_term = DoneTerm(
        func=mdp.nan_detection,
        time_out=False,
    )

# Curriculums

@configclass
class CurriculumsCfg:
    """Curriculum terms for the MDP."""

    terrain_levels = CurrTerm(
        func=mdp.terrain_levels_vel,
        params={"command_name": "twist"},
    )
    command_vel = CurrTerm(
        func=mdp.commands_vel,
        params={
            "command_name": "twist",
            "velocity_stages": [
                {
                    "step": 0,
                    "lin_vel_x": (-0.3, 0.5),
                    "lin_vel_y": (-0.3, 0.3),
                    "ang_vel_z": (-0.5, 0.5),
                },
                {
                    "step": 5000 * 32,
                    "lin_vel_x": (-0.5, 1.0),
                    "lin_vel_y": (-0.5, 0.5),
                    "ang_vel_z": (-0.8, 0.8),
                },
                # {
                #     "step": 10000 * 32,
                #     "lin_vel_x": (-1.0, 1.5),
                #     "lin_vel_y": (-0.5, 0.5),
                #     "ang_vel_z": (-1.0, 1.0),
                # },
            ],
        },
    )

    robot_vel = CurrTerm(func=mdp.log_robot_vel)



##
# Environment configuration
##


@configclass
class VinRslRlEnvCfg(ManagerBasedRLEnvCfg):
    # Scene settings
    scene: VinRslRlSceneCfg = VinRslRlSceneCfg(num_envs=4096, env_spacing=4.0)
    # Basic settings
    observations: ObservationsCfg = ObservationsCfg()
    actions: ActionsCfg = ActionsCfg()
    commands: CommandsCfg = CommandsCfg()
    events: EventCfg = EventCfg()
    # MDP settings
    rewards: RewardsCfg = RewardsCfg()
    terminations: TerminationsCfg = TerminationsCfg()
    curriculum: CurriculumsCfg = CurriculumsCfg()

    # Post initialization
    def __post_init__(self) -> None:
        """Post initialization."""
        self.scene.robot = CHAOS_CFG.replace(prim_path="{ENV_REGEX_NS}/Robot")
        # general settings
        self.decimation = 4
        self.episode_length_s = 20
        # viewer settings
        self.viewer.eye = (8.0, 0.0, 5.0)
        # simulation settings
        self.sim.dt = 0.005
        self.sim.render_interval = self.decimation


@configclass
class VinRslRlFlatEnvCfg(VinRslRlEnvCfg):
    """Lighter variant that uses a flat plane instead of generated terrains."""

    scene: VinRslRlFlatSceneCfg = VinRslRlFlatSceneCfg(num_envs=4096, env_spacing=4.0)


@configclass
class VinRslRlPlayEnvCfg(VinRslRlEnvCfg):
    """Play variant for visualization and policy rollout."""

    scene: VinRslRlSceneCfg = VinRslRlSceneCfg(num_envs=32, env_spacing=4.0)

    def __post_init__(self) -> None:
        super().__post_init__()
        self.episode_length_s = int(1e9)
        self.observations.policy.enable_corruption = True # enable noise when play
        self.events.push_robot = None
        self.curriculum.terrain_levels = None
        self.curriculum.command_vel = None
        self.viewer.origin_type = "asset_body"
        self.viewer.asset_name = "robot"
        self.viewer.body_name = ROOT_BODY
        self.viewer.eye = (2.0, 2.0, 1.2)
        self.viewer.lookat = (0.0, 0.0, 0.35)
        self.commands.twist.ranges.lin_vel_x = (0.4, 0.4)
        self.commands.twist.ranges.lin_vel_y = (-0.0, 0.0)
        self.commands.twist.ranges.ang_vel_z = (-0.0, 0.0)
        if self.scene.terrain.terrain_generator is not None:
            self.scene.terrain.terrain_generator.curriculum = False
            self.scene.terrain.terrain_generator.num_cols = 10
            self.scene.terrain.terrain_generator.num_rows = 10
            self.scene.terrain.terrain_generator.border_width = 10.0


@configclass
class VinRslRlFlatPlayEnvCfg(VinRslRlFlatEnvCfg):
    """Flat-ground play variant for lightweight policy rollout."""

    scene: VinRslRlFlatSceneCfg = VinRslRlFlatSceneCfg(num_envs=32, env_spacing=4.0)

    def __post_init__(self) -> None:
        super().__post_init__()
        self.episode_length_s = int(1e9)
        self.observations.policy.enable_corruption = True
        self.events.push_robot = None
        self.curriculum.terrain_levels = None
        self.curriculum.command_vel = None
        self.viewer.origin_type = "asset_body"
        self.viewer.asset_name = "robot"
        self.viewer.body_name = ROOT_BODY
        self.viewer.eye = (2.0, 2.0, 1.2)
        self.viewer.lookat = (0.0, 0.0, 0.35)
        self.commands.twist.ranges.lin_vel_x = (0.4, 0.4)
        self.commands.twist.ranges.lin_vel_y = (-0.0, 0.0)
        self.commands.twist.ranges.ang_vel_z = (-0.0, 0.0)



