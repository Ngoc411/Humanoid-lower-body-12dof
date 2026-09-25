# Copyright (c) 2022-2026, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

from __future__ import annotations

from typing import TYPE_CHECKING

import torch

from isaaclab.assets import Articulation
from isaaclab.managers import SceneEntityCfg

if TYPE_CHECKING:
    from isaaclab.envs import ManagerBasedRLEnv


def nan_detection(
    env: ManagerBasedRLEnv,
    asset_cfg: SceneEntityCfg = SceneEntityCfg("robot"),
) -> torch.Tensor:
    """Terminate environments whose robot state contains NaN or Inf values."""
    asset: Articulation = env.scene[asset_cfg.name]
    state_tensors = (
        asset.data.root_pos_w,
        asset.data.root_quat_w,
        asset.data.root_lin_vel_w,
        asset.data.root_ang_vel_w,
        asset.data.joint_pos,
        asset.data.joint_vel,
    )
    invalid = torch.zeros(env.num_envs, dtype=torch.bool, device=env.device)
    for tensor in state_tensors:
        invalid |= ~torch.isfinite(tensor.flatten(start_dim=1)).all(dim=1)
    return invalid