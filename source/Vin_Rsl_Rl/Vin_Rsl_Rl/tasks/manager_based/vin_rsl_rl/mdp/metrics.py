# Copyright (c) 2022-2026, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Raw locomotion metrics for logging and tuning."""

from __future__ import annotations

from typing import TYPE_CHECKING

import torch

from isaaclab.assets import Articulation
from isaaclab.managers import SceneEntityCfg
from isaaclab.utils.math import quat_apply_inverse

if TYPE_CHECKING:
    from isaaclab.envs import ManagerBasedRLEnv

_DEFAULT_ASSET_CFG = SceneEntityCfg("robot")


def lin_vel_error_x(
    env: ManagerBasedRLEnv,
    command_name: str,
    asset_cfg: SceneEntityCfg = _DEFAULT_ASSET_CFG,
) -> torch.Tensor:
    """Absolute forward velocity tracking error in m/s."""
    asset: Articulation = env.scene[asset_cfg.name]
    command = env.command_manager.get_command(command_name)
    if command is None:
        raise ValueError(f"Command '{command_name}' not found.")
    return torch.abs(command[:, 0] - asset.data.root_lin_vel_b[:, 0])


def lin_vel_error_y(
    env: ManagerBasedRLEnv,
    command_name: str,
    asset_cfg: SceneEntityCfg = _DEFAULT_ASSET_CFG,
) -> torch.Tensor:
    """Absolute lateral velocity tracking error in m/s."""
    asset: Articulation = env.scene[asset_cfg.name]
    command = env.command_manager.get_command(command_name)
    if command is None:
        raise ValueError(f"Command '{command_name}' not found.")
    return torch.abs(command[:, 1] - asset.data.root_lin_vel_b[:, 1])


def ang_vel_error_z(
    env: ManagerBasedRLEnv,
    command_name: str,
    asset_cfg: SceneEntityCfg = _DEFAULT_ASSET_CFG,
) -> torch.Tensor:
    """Absolute yaw-rate tracking error in rad/s."""
    asset: Articulation = env.scene[asset_cfg.name]
    command = env.command_manager.get_command(command_name)
    if command is None:
        raise ValueError(f"Command '{command_name}' not found.")
    return torch.abs(command[:, 2] - asset.data.root_ang_vel_b[:, 2])


def body_link_roll(
    env: ManagerBasedRLEnv,
    asset_cfg: SceneEntityCfg,
) -> torch.Tensor:
    """Absolute body-link roll angle in rad, measured from projected gravity."""
    asset: Articulation = env.scene[asset_cfg.name]
    link_quat_w = asset.data.body_quat_w[:, asset_cfg.body_ids[0], :]
    gravity_b = quat_apply_inverse(link_quat_w, asset.data.GRAVITY_VEC_W)
    return torch.abs(torch.atan2(gravity_b[:, 1], -gravity_b[:, 2]))


def body_link_pitch(
    env: ManagerBasedRLEnv,
    asset_cfg: SceneEntityCfg,
) -> torch.Tensor:
    """Absolute body-link pitch angle in rad, measured from projected gravity."""
    asset: Articulation = env.scene[asset_cfg.name]
    link_quat_w = asset.data.body_quat_w[:, asset_cfg.body_ids[0], :]
    gravity_b = quat_apply_inverse(link_quat_w, asset.data.GRAVITY_VEC_W)
    return torch.abs(torch.atan2(-gravity_b[:, 0], -gravity_b[:, 2]))


def log_field_metrics(
    env: ManagerBasedRLEnv,
    command_name: str,
    torso_asset_cfg: SceneEntityCfg,
    asset_cfg: SceneEntityCfg = _DEFAULT_ASSET_CFG,
) -> torch.Tensor:
    """Log MJLab-style field metrics into env.extras without changing rewards."""
    metrics = {
        "lin_vel_error_x": lin_vel_error_x(env, command_name, asset_cfg),
        "lin_vel_error_y": lin_vel_error_y(env, command_name, asset_cfg),
        "ang_vel_error_z": ang_vel_error_z(env, command_name, asset_cfg),
        "waist_roll": body_link_roll(env, torso_asset_cfg),
        "waist_pitch": body_link_pitch(env, torso_asset_cfg),
    }
    log = env.extras.setdefault("log", {})
    for name, value in metrics.items():
        log[f"Episode_Metrics/{name}"] = value.mean()
    return torch.zeros(env.num_envs, device=env.device)

