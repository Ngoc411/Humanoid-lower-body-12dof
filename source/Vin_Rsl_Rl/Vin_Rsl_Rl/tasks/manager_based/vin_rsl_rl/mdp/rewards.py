# Copyright (c) 2022-2026, The Isaac Lab Project Developers (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

from __future__ import annotations

from typing import TYPE_CHECKING

import torch

from isaaclab.utils.math import wrap_to_pi


import isaaclab.utils.math as math_utils
from isaaclab.utils.math import quat_apply, quat_apply_inverse
from isaaclab.utils.string import resolve_matching_names_values
import isaaclab.utils.string as string_utils
from isaaclab.assets import Articulation, RigidObject
from isaaclab.managers import SceneEntityCfg
from isaaclab.sensors import ContactSensor, FrameTransformer, RayCaster
from isaaclab.managers import RewardTermCfg 
from isaaclab.managers.manager_base import ManagerTermBase

if TYPE_CHECKING:
    from isaaclab.envs import ManagerBasedRLEnv

_DEFAULT_ASSET_CFG = SceneEntityCfg("robot")



def joint_pos_target_l2(env: ManagerBasedRLEnv, target: float, asset_cfg: SceneEntityCfg) -> torch.Tensor:
    """Penalize joint position deviation from a target value."""
    # extract the used quantities (to enable type-hinting)
    asset: Articulation = env.scene[asset_cfg.name]
    # wrap the joint positions to (-pi, pi)
    joint_pos = wrap_to_pi(asset.data.joint_pos[:, asset_cfg.joint_ids])
    # compute the reward
    return torch.sum(torch.square(joint_pos - target), dim=1)







def _track_velocity_axis(
    env: ManagerBasedRLEnv,
    std_list: list[float],
    vel_list: list[float],
    command_name: str,
    cmd_axis: int,
    actual_vel_axis: torch.Tensor,
    add_penalize: bool,
    penalize_scale: float,
) -> torch.Tensor:
    """Exponential-kernel reward for a single velocity axis.

    Returns exp(-eÂ²/ÏƒÂ²) in [0, 1], or (1+scale)*exp(-eÂ²/ÏƒÂ²) - scale in [-scale, 1]
    when add_penalize is True.

    Ïƒ is a step function of |command|: segment [vel_list[i], vel_list[i+1]) maps
    to std_list[i], matching the bucketize convention used by body_roll_pitch_penalty.
    vel_list must have exactly len(std_list)+1 entries.
    """
    if len(vel_list) != len(std_list) + 1:
        raise ValueError(
            f"vel_list must have len(std_list)+1 entries, got vel_list={len(vel_list)} vs std_list={len(std_list)}"
        )
    command = env.command_manager.get_command(command_name)
    if command is None:
        raise ValueError(f"Command '{command_name}' not found.")
    cmd_abs = torch.abs(command[:, cmd_axis])
    boundaries = torch.tensor(vel_list[1:], device=cmd_abs.device, dtype=cmd_abs.dtype)
    bin_idx = torch.bucketize(cmd_abs.contiguous(), boundaries).clamp(
        max=len(std_list) - 1
    )
    std_t = torch.tensor(std_list, device=cmd_abs.device, dtype=cmd_abs.dtype)[bin_idx]
    error = torch.square(command[:, cmd_axis] - actual_vel_axis)
    r = torch.exp(-error / std_t**2)
    if add_penalize:
        return (1.0 + penalize_scale) * r - penalize_scale
    return r



def track_linear_x(
    env: ManagerBasedRLEnv,
    std_list: list[float],
    vel_list: list[float],
    command_name: str,
    asset_cfg: SceneEntityCfg = _DEFAULT_ASSET_CFG,
    add_penalize: bool = False,
    penalize_scale: float = 1.0,
) -> torch.Tensor:
    """Reward tracking of linear x velocity command using an exponential kernel."""
    asset: Articulation = env.scene[asset_cfg.name]
    return _track_velocity_axis(
        env,
        std_list,
        vel_list,
        command_name,
        0,
        asset.data.root_lin_vel_b[:, 0],
        add_penalize,
        penalize_scale,
    )


def track_linear_y(
    env: ManagerBasedRLEnv,
    std_list: list[float],
    vel_list: list[float],
    command_name: str,
    asset_cfg: SceneEntityCfg = _DEFAULT_ASSET_CFG,
    add_penalize: bool = False,
    penalize_scale: float = 1.0,
) -> torch.Tensor:
    """Reward tracking of linear y velocity command using an exponential kernel."""
    asset: Articulation = env.scene[asset_cfg.name]
    return _track_velocity_axis(
        env,
        std_list,
        vel_list,
        command_name,
        1,
        asset.data.root_lin_vel_b[:, 1],
        add_penalize,
        penalize_scale,
    )


def track_angular_z(
    env: ManagerBasedRLEnv,
    std_list: list[float],
    vel_list: list[float],
    command_name: str,
    asset_cfg: SceneEntityCfg = _DEFAULT_ASSET_CFG,
    add_penalize: bool = False,
    penalize_scale: float = 1.0,
) -> torch.Tensor:
    """Reward tracking of angular z velocity command using an exponential kernel."""
    asset: Articulation = env.scene[asset_cfg.name]
    return _track_velocity_axis(
        env,
        std_list,
        vel_list,
        command_name,
        2,
        asset.data.root_ang_vel_b[:, 2],
        add_penalize,
        penalize_scale,
    )


def lin_vel_z_l2(
    env: ManagerBasedRLEnv,
    asset_cfg: SceneEntityCfg = _DEFAULT_ASSET_CFG,
) -> torch.Tensor:
    """Penalize z-axis base linear velocity using L2 squared kernel."""
    # extract the used quantities (to enable type-hinting)
    asset: Articulation = env.scene[asset_cfg.name]
    return torch.square(asset.data.root_lin_vel_b[:, 2])


def ang_vel_xy_l2(
    env: ManagerBasedRLEnv,
    asset_cfg: SceneEntityCfg = _DEFAULT_ASSET_CFG,
) -> torch.Tensor:
    """Penalize xy-axis base angular velocity using L2 squared kernel."""
    # extract the used quantities (to enable type-hinting)
    asset: Articulation = env.scene[asset_cfg.name]
    return torch.sum(torch.square(asset.data.root_ang_vel_b[:, :2]), dim=1)


def link_orientation(
    env: ManagerBasedRLEnv,
    asset_cfg: SceneEntityCfg = SceneEntityCfg("robot"),
) -> torch.Tensor:
    """Penalize non-flat link orientation using L2 squared kernel."""
    asset: Articulation = env.scene[asset_cfg.name]
    link_quat = asset.data.body_quat_w[:, asset_cfg.body_ids[0], :]
    link_projected_gravity = quat_apply_inverse(link_quat, asset.data.GRAVITY_VEC_W)
    return torch.sum(torch.square(link_projected_gravity[:, :2]), dim=1)


def body_roll_pitch_penalty(
    env: ManagerBasedRLEnv,
    weight_list: list[float] | None = None,
    velocity_list: list[float] | None = None,
    command_name: str | None = None,
    asset_cfg: SceneEntityCfg = _DEFAULT_ASSET_CFG,
) -> torch.Tensor:
    """Penalize roll and pitch of a body link relative to the world frame.

    Projects the gravity vector into the body frame (via the body's world-frame
    quaternion) and derives roll/pitch angles via atan2 â€” the same convention
    used by ``imu_roll_pitch_penalty`` for the root link.

    Use ``asset_cfg.body_names = ["waist_roll_link"]`` to target that body.
    """
    asset: Articulation = env.scene[asset_cfg.name]
    if not asset_cfg.body_ids or len(asset_cfg.body_ids) != 1:
        raise ValueError(
            f"body_roll_pitch_penalty expects exactly one body, got body_ids={asset_cfg.body_ids}"
        )
    body_quat_w = asset.data.body_quat_w[:, asset_cfg.body_ids[0], :]  # [B, 4]
    gravity_w = asset.data.GRAVITY_VEC_W
    projected_gravity_b = quat_apply_inverse(body_quat_w, gravity_w)  # [B, 3]
    roll = torch.atan2(projected_gravity_b[:, 1], -projected_gravity_b[:, 2])
    pitch = torch.atan2(-projected_gravity_b[:, 0], -projected_gravity_b[:, 2])
    roll_pitch_error = torch.square(roll) + torch.square(pitch)
    if weight_list is not None and velocity_list is not None:
        command = env.command_manager.get_command(command_name)
        assert command is not None, f"Command '{command_name}' not found."
        vx_cmd = torch.abs(command[:, 0])
        boundaries = torch.tensor(
            velocity_list[1:], device=vx_cmd.device, dtype=vx_cmd.dtype
        )
        bin_idx = torch.bucketize(vx_cmd.contiguous(), boundaries).clamp(
            max=len(weight_list) - 1
        )
        weight_t = torch.tensor(weight_list, device=vx_cmd.device, dtype=vx_cmd.dtype)
        weights = weight_t[bin_idx]
        roll_pitch_error = roll_pitch_error * weights
    return roll_pitch_error


class variable_posture(ManagerTermBase):
    """Penalize deviation from default pose with speed-dependent tolerance.

    Uses per-joint standard deviations to control how much each joint can deviate
    from default pose. Smaller std = stricter (less deviation allowed), larger
    std = more forgiving. The reward is: exp(-mean(errorÂ² / stdÂ²))

    Three speed regimes (based on linear + angular command velocity):
      - std_standing (speed < walking_threshold): Tight tolerance for holding pose.
      - std_walking (walking_threshold <= speed < running_threshold): Moderate.
      - std_running (speed >= running_threshold): Loose tolerance for large motion.

    Tune std values per joint based on how much motion that joint needs at each
    speed. Map joint name patterns to std values, e.g. {".*knee.*": 0.35}.
    """

    def __init__(self, cfg: RewardTermCfg, env: ManagerBasedRLEnv):
        super().__init__(cfg, env)
        asset: Articulation = env.scene[cfg.params["asset_cfg"].name]
        default_joint_pos = asset.data.default_joint_pos
        assert default_joint_pos is not None
        self.default_joint_pos = default_joint_pos

        _, joint_names = asset.find_joints(cfg.params["asset_cfg"].joint_names)

        _, _, std_standing = resolve_matching_names_values(
            data=cfg.params["std_standing"],
            list_of_strings=joint_names,
        )
        self.std_standing = torch.tensor(
            std_standing, device=env.device, dtype=torch.float32
        )

        _, _, std_walking = resolve_matching_names_values(
            data=cfg.params["std_walking"],
            list_of_strings=joint_names,
        )
        self.std_walking = torch.tensor(
            std_walking, device=env.device, dtype=torch.float32
        )

        _, _, std_running = resolve_matching_names_values(
            data=cfg.params["std_running"],
            list_of_strings=joint_names,
        )
        self.std_running = torch.tensor(
            std_running, device=env.device, dtype=torch.float32
        )

    def __call__(
        self,
        env: ManagerBasedRLEnv,
        std_standing,
        std_walking,
        std_running,
        asset_cfg: SceneEntityCfg,
        command_name: str,
        walking_threshold: float = 0.5,
        running_threshold: float = 1.5,
    ) -> torch.Tensor:
        del std_standing, std_walking, std_running  # Unused.

        asset: Articulation = env.scene[asset_cfg.name]
        command = env.command_manager.get_command(command_name)
        assert command is not None

        linear_speed = torch.norm(command[:, :2], dim=1)
        angular_speed = torch.abs(command[:, 2])
        total_speed = linear_speed + angular_speed

        standing_mask = (total_speed < walking_threshold).float()
        walking_mask = (
            (total_speed >= walking_threshold) & (total_speed < running_threshold)
        ).float()
        running_mask = (total_speed >= running_threshold).float()

        std = (
            self.std_standing * standing_mask.unsqueeze(1)
            + self.std_walking * walking_mask.unsqueeze(1)
            + self.std_running * running_mask.unsqueeze(1)
        )

        current_joint_pos = asset.data.joint_pos[:, asset_cfg.joint_ids]
        desired_joint_pos = self.default_joint_pos[:, asset_cfg.joint_ids]
        error_squared = torch.square(current_joint_pos - desired_joint_pos)

        return torch.exp(-torch.mean(error_squared / (std**2), dim=1))


def feet_air_time_positive_biped(
    env: ManagerBasedRLEnv,
    command_name: str,
    threshold: float,
    sensor_name: str,
) -> torch.Tensor:
    """Reward long steps taken by the feet for bipeds.

    This function rewards the agent for taking steps up to a specified threshold and also keep one foot at
    a time in the air.

    If the commands are small (i.e. the agent is not supposed to take a step), then the reward is zero.
    """
    contact_sensor: ContactSensor = env.scene[sensor_name]
    air_time = contact_sensor.data.current_air_time
    contact_time = contact_sensor.data.current_contact_time
    # compute the reward
    in_contact = contact_time > 0.0
    in_mode_time = torch.where(in_contact, contact_time, air_time)
    single_stance = torch.sum(in_contact.int(), dim=1) == 1
    reward = torch.min(
        torch.where(single_stance.unsqueeze(-1), in_mode_time, 0.0), dim=1
    )[0]
    reward = torch.clamp(reward, max=threshold)
    # no reward for zero command
    reward *= (
        torch.norm(env.command_manager.get_command(command_name)[:, :2], dim=1) > 0.1
    )
    return reward


def feet_clearance(
    env: ManagerBasedRLEnv,
    target_height: float,
    command_name: str | None = None,
    command_threshold: float = 0.1,
    asset_cfg: SceneEntityCfg = _DEFAULT_ASSET_CFG,
    foot_offset: tuple[float, float, float] | None = None,
) -> torch.Tensor:
    """Penalize deviation from target foot-link height, weighted by foot xy velocity."""
    asset: Articulation = env.scene[asset_cfg.name]
    foot_pos_w = asset.data.body_pos_w[:, asset_cfg.body_ids, :]
    foot_vel_w = asset.data.body_lin_vel_w[:, asset_cfg.body_ids, :]
    if foot_offset is not None:
        offset_b = torch.tensor(foot_offset, device=foot_pos_w.device, dtype=foot_pos_w.dtype)
        offset_b = offset_b.expand(foot_pos_w.shape[0], foot_pos_w.shape[1], 3)
        foot_quat_w = asset.data.body_quat_w[:, asset_cfg.body_ids, :]
        offset_w = quat_apply(foot_quat_w.reshape(-1, 4), offset_b.reshape(-1, 3)).reshape_as(foot_pos_w)
        foot_pos_w = foot_pos_w + offset_w
        foot_ang_vel_w = asset.data.body_ang_vel_w[:, asset_cfg.body_ids, :]
        foot_vel_w = foot_vel_w + torch.cross(foot_ang_vel_w, offset_w, dim=-1)
    foot_z = foot_pos_w[:, :, 2]
    foot_vel_xy = foot_vel_w[:, :, :2]
    vel_norm = torch.norm(foot_vel_xy, dim=-1)
    delta = torch.abs(foot_z - target_height)
    cost = torch.sum(delta * vel_norm, dim=1)
    if command_name is not None:
        command = env.command_manager.get_command(command_name)
        if command is not None:
            total_command = torch.norm(command[:, :2], dim=1) + torch.abs(command[:, 2])
            cost = cost * (total_command > command_threshold).float()
    return cost


def feet_slip(
    env: ManagerBasedRLEnv,
    sensor_name: str,
    command_name: str,
    command_threshold: float = 0.01,
    asset_cfg: SceneEntityCfg = _DEFAULT_ASSET_CFG,
    foot_offset: tuple[float, float, float] | None = None,
) -> torch.Tensor:
    """Penalize foot sliding xy velocity while the foot contact sensor is active."""
    asset: Articulation = env.scene[asset_cfg.name]
    contact_sensor: ContactSensor = env.scene[sensor_name]
    command = env.command_manager.get_command(command_name)
    assert command is not None
    total_command = torch.norm(command[:, :2], dim=1) + torch.abs(command[:, 2])
    active = (total_command > command_threshold).float()
    in_contact = (contact_sensor.data.current_contact_time > 0.0).float()
    foot_vel_w = asset.data.body_lin_vel_w[:, asset_cfg.body_ids, :]
    if foot_offset is not None:
        foot_pos_w = asset.data.body_pos_w[:, asset_cfg.body_ids, :]
        offset_b = torch.tensor(foot_offset, device=foot_vel_w.device, dtype=foot_vel_w.dtype)
        offset_b = offset_b.expand(foot_vel_w.shape[0], foot_vel_w.shape[1], 3)
        foot_quat_w = asset.data.body_quat_w[:, asset_cfg.body_ids, :]
        offset_w = quat_apply(foot_quat_w.reshape(-1, 4), offset_b.reshape(-1, 3)).reshape_as(foot_pos_w)
        foot_ang_vel_w = asset.data.body_ang_vel_w[:, asset_cfg.body_ids, :]
        foot_vel_w = foot_vel_w + torch.cross(foot_ang_vel_w, offset_w, dim=-1)
    foot_vel_xy = foot_vel_w[:, :, :2]
    vel_xy_norm = torch.norm(foot_vel_xy, dim=-1)
    cost = torch.sum(torch.square(vel_xy_norm) * in_contact, dim=1) * active
    num_in_contact = torch.sum(in_contact)
    mean_slip_vel = torch.sum(vel_xy_norm * in_contact) / torch.clamp(num_in_contact, min=1)
    env.extras["log"]["Episode_Metrics/slip_velocity_mean"] = mean_slip_vel
    return cost


def feet_close_xy_gauss(
    env: ManagerBasedRLEnv,
    threshold: float,
    asset_cfg: SceneEntityCfg = _DEFAULT_ASSET_CFG,
    std: float = 0.1,
    foot_offset: tuple[float, float, float] | None = None,
) -> torch.Tensor:
    """Penalize when feet are too close together in the y distance."""
    asset: Articulation = env.scene[asset_cfg.name]
    body_pos_w = asset.data.body_pos_w[:, asset_cfg.body_ids, :]
    if foot_offset is not None:
        offset_b = torch.tensor(foot_offset, device=body_pos_w.device, dtype=body_pos_w.dtype)
        offset_b = offset_b.expand(body_pos_w.shape[0], body_pos_w.shape[1], 3)
        body_quat_w = asset.data.body_quat_w[:, asset_cfg.body_ids, :]
        body_pos_w = body_pos_w + quat_apply(
            body_quat_w.reshape(-1, 4), offset_b.reshape(-1, 3)
        ).reshape_as(body_pos_w)

    left_foot_xy = body_pos_w[:, 0, :2]
    right_foot_xy = body_pos_w[:, 1, :2]
    heading_w = asset.data.heading_w

    cos_heading = torch.cos(heading_w)
    sin_heading = torch.sin(heading_w)

    left_y = -sin_heading * left_foot_xy[:, 0] + cos_heading * left_foot_xy[:, 1]
    right_y = -sin_heading * right_foot_xy[:, 0] + cos_heading * right_foot_xy[:, 1]
    feet_distance_y = torch.abs(left_y - right_y)

    return torch.exp(-torch.clamp(threshold - feet_distance_y, min=0.0) / std**2) - 1


def body_angular_velocity_penalty(
    env: ManagerBasedRLEnv,
    asset_cfg: SceneEntityCfg = _DEFAULT_ASSET_CFG,
) -> torch.Tensor:
    """Penalize excessive body angular velocities."""
    asset: Articulation = env.scene[asset_cfg.name]
    ang_vel = asset.data.body_ang_vel_w[:, asset_cfg.body_ids, :]
    ang_vel = ang_vel.squeeze(1)
    ang_vel_xy = ang_vel[:, :2]  # Don't penalize z-angular velocity.
    return torch.sum(torch.square(ang_vel_xy), dim=1)


def stand_still(
    env: ManagerBasedRLEnv,
    command_name: str,
    command_threshold: float = 0.1,
    asset_cfg: SceneEntityCfg = _DEFAULT_ASSET_CFG,
) -> torch.Tensor:
    asset: Articulation = env.scene[asset_cfg.name]
    diff_angle = (
        asset.data.joint_pos[:, asset_cfg.joint_ids]
        - asset.data.default_joint_pos[:, asset_cfg.joint_ids]
    )
    reward = torch.sum(torch.square(diff_angle), dim=1)
    if command_name is not None:
        command = env.command_manager.get_command(command_name)
        if command is not None:
            linear_norm = torch.norm(command[:, :2], dim=1)
            angular_norm = torch.abs(command[:, 2])
            total_command = linear_norm + angular_norm
            scale = (total_command <= command_threshold).float()
            reward *= scale
    return reward


def dont_wait(
    env: ManagerBasedRLEnv,
    command_name: str,
    cmd_threshold: float = 0.1,
    asset_cfg: SceneEntityCfg = _DEFAULT_ASSET_CFG,
) -> torch.Tensor:
    """Penalize standing still when there is a forward velocity command."""
    asset: Articulation = env.scene[asset_cfg.name]
    lin_vel_cmd_x = env.command_manager.get_command(command_name)[:, 0]
    lin_vel_x = asset.data.root_lin_vel_b[:, 0]

    v1 = cmd_threshold * 0.5
    v2 = 0.0  # stationary
    v3 = -cmd_threshold * 0.5

    return (lin_vel_cmd_x > cmd_threshold) * (
        (lin_vel_x < v1).float() + (lin_vel_x < v2).float() + (lin_vel_x < v3).float()
    )


def knee_joint_motion(
    env: ManagerBasedRLEnv,
    reward_limit: float,
    command_name: str | None = None,
    command_threshold: float = 0.1,
    asset_cfg: SceneEntityCfg = _DEFAULT_ASSET_CFG,
) -> torch.Tensor:
    """Reward knee joint movement (velocity magnitude), capped at reward_limit.

    Encourages the knee joints to actively move without over-rewarding
    extreme velocities. The sum of squared knee joint velocities is clamped
    to reward_limit, so the reward saturates once the knees move sufficiently.

    Args:
        reward_limit: Maximum reward per step (clamp ceiling).
        command_name: If set, gate reward by command magnitude > command_threshold.
        command_threshold: Minimum total command norm to activate reward.
        asset_cfg: Asset config with joint_ids resolved to knee joints.
    """
    asset: Articulation = env.scene[asset_cfg.name]
    knee_vel = asset.data.joint_vel[:, asset_cfg.joint_ids]  # [B, N]
    motion = torch.sum(torch.square(knee_vel), dim=1)  # [B]
    reward = torch.clamp(motion, max=reward_limit)
    if command_name is not None:
        command = env.command_manager.get_command(command_name)
        if command is not None:
            linear_norm = torch.norm(command[:, :2], dim=1)
            angular_norm = torch.abs(command[:, 2])
            total_command = linear_norm + angular_norm
            reward = reward * (total_command > command_threshold).float()
    return reward


# hip pitch +- 0.12rad/s - 0.4m/s
# def self_collision_cost(
#     env: ManagerBasedRLEnv,
#     sensor_name: str,
#     force_threshold: float = 10.0,
# ) -> torch.Tensor:
#     """Penalize environments with non-foot/contact collision forces over a threshold."""
#     sensor: ContactSensor = env.scene[sensor_name]
#     force_mag = torch.norm(sensor.data.net_forces_w, dim=-1)
#     return (force_mag > force_threshold).any(dim=1).float()

# def dof_vel(
#     env: ManagerBasedRLEnv,
# ):
