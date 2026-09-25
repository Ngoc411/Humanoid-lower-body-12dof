from __future__ import annotations

from collections.abc import Sequence
from typing import TYPE_CHECKING, TypedDict

import torch

from isaaclab.assets import Articulation
from isaaclab.managers import SceneEntityCfg
from isaaclab.terrains import TerrainImporter

if TYPE_CHECKING:
    from isaaclab.envs import ManagerBasedRLEnv

_DEFAULT_ASSET_CFG = SceneEntityCfg("robot")


def terrain_levels_vel(
    env: ManagerBasedRLEnv,
    env_ids: Sequence[int],
    command_name: str,
    asset_cfg: SceneEntityCfg = _DEFAULT_ASSET_CFG,
) -> torch.Tensor:
    asset: Articulation = env.scene[asset_cfg.name]
    terrain: TerrainImporter = env.scene.terrain
    if terrain.cfg.terrain_generator is None or not hasattr(terrain, "terrain_levels"):
        return torch.tensor(0.0, device=env.device)

    command = env.command_manager.get_command(command_name)
    assert command is not None

    distance = torch.norm(
        asset.data.root_pos_w[env_ids, :2] - env.scene.env_origins[env_ids, :2],
        dim=1,
    )
    move_up = distance > terrain.cfg.terrain_generator.size[0] / 2
    move_down = distance < torch.norm(command[env_ids, :2], dim=1) * env.max_episode_length_s * 0.5
    move_down *= ~move_up

    terrain.update_env_origins(env_ids, move_up, move_down)
    return torch.mean(terrain.terrain_levels.float())


class VelocityStage(TypedDict, total=False):
    step: int
    lin_vel_x: tuple[float, float]
    lin_vel_y: tuple[float, float]
    ang_vel_z: tuple[float, float]


def commands_vel(
    env: ManagerBasedRLEnv,
    env_ids: Sequence[int],
    command_name: str,
    velocity_stages: list[VelocityStage],
) -> dict[str, torch.Tensor]:
    del env_ids
    command_term = env.command_manager.get_term(command_name)
    assert command_term is not None
    ranges = command_term.cfg.ranges

    for stage in velocity_stages:
        if env.common_step_counter >= stage["step"]:
            if "lin_vel_x" in stage:
                ranges.lin_vel_x = stage["lin_vel_x"]
            if "lin_vel_y" in stage:
                ranges.lin_vel_y = stage["lin_vel_y"]
            if "ang_vel_z" in stage:
                ranges.ang_vel_z = stage["ang_vel_z"]

    return {
        "lin_vel_x_min": torch.tensor(ranges.lin_vel_x[0], device=env.device),
        "lin_vel_x_max": torch.tensor(ranges.lin_vel_x[1], device=env.device),
        "lin_vel_y_min": torch.tensor(ranges.lin_vel_y[0], device=env.device),
        "lin_vel_y_max": torch.tensor(ranges.lin_vel_y[1], device=env.device),
        "ang_vel_z_min": torch.tensor(ranges.ang_vel_z[0], device=env.device),
        "ang_vel_z_max": torch.tensor(ranges.ang_vel_z[1], device=env.device),
    }


def log_robot_vel(
    env: ManagerBasedRLEnv, env_ids: Sequence[int], asset_cfg: SceneEntityCfg = SceneEntityCfg("robot")
) -> dict:
    asset: Articulation = env.scene[asset_cfg.name]
    return {
        "x": asset.data.root_lin_vel_b[env_ids, 0].mean(),
        "y": asset.data.root_lin_vel_b[env_ids, 1].mean(),
    }
