from __future__ import annotations

import torch
from typing import TYPE_CHECKING

import isaaclab.utils.math as math_utils
import isaaclab.utils.string as string_utils
from isaaclab.assets import Articulation, RigidObject
from isaaclab.managers import SceneEntityCfg
from isaaclab.sensors import ContactSensor, FrameTransformer, RayCaster

if TYPE_CHECKING:
    from isaaclab.envs import ManagerBasedEnv

_DEFAULT_ASSET_CFG = SceneEntityCfg("robot")


def phase(env: ManagerBasedEnv, period: float, command_name: str) -> torch.Tensor:
    global_phase = (env.episode_length_buf * env.step_dt) % period / period
    phase = torch.zeros(env.num_envs, 2, device=env.device)
    phase[:, 0] = torch.sin(global_phase * torch.pi * 2.0)
    phase[:, 1] = torch.cos(global_phase * torch.pi * 2.0)
    stand_mask = torch.linalg.norm(env.command_manager.get_command(command_name), dim=1) < 0.1
    phase = torch.where(stand_mask.unsqueeze(1), torch.zeros_like(phase), phase)
    return phase


def foot_height(
  env: ManagerBasedEnv,
  asset_cfg: SceneEntityCfg = _DEFAULT_ASSET_CFG,
  foot_offset: tuple[float, float, float] | None = None,
) -> torch.Tensor:
  asset: Articulation | RigidObject = env.scene[asset_cfg.name]
  foot_pos_w = asset.data.body_pos_w[:, asset_cfg.body_ids, :]
  if foot_offset is not None:
      offset_b = torch.tensor(foot_offset, device=foot_pos_w.device, dtype=foot_pos_w.dtype)
      offset_b = offset_b.expand(foot_pos_w.shape[0], foot_pos_w.shape[1], 3)
      foot_quat_w = asset.data.body_quat_w[:, asset_cfg.body_ids, :]
      foot_pos_w = foot_pos_w + math_utils.quat_apply(
          foot_quat_w.reshape(-1, 4), offset_b.reshape(-1, 3)
      ).reshape_as(foot_pos_w)
  return foot_pos_w[:, :, 2]  # (num_envs, num_bodies)


def foot_air_time(env: ManagerBasedEnv, sensor_name: str) -> torch.Tensor:
  sensor: ContactSensor = env.scene[sensor_name]
  sensor_data = sensor.data
  current_air_time = sensor_data.current_air_time
  assert current_air_time is not None
  return current_air_time


def foot_contact(env: ManagerBasedEnv, sensor_name: str) -> torch.Tensor:
  sensor: ContactSensor = env.scene[sensor_name]
  return (sensor.data.current_contact_time > 0.0).float()


def foot_contact_forces(env: ManagerBasedEnv, sensor_name: str) -> torch.Tensor:
  sensor: ContactSensor = env.scene[sensor_name]
  forces_flat = sensor.data.net_forces_w.flatten(start_dim=1)
  return torch.sign(forces_flat) * torch.log1p(torch.abs(forces_flat))


def _minjerk(u: torch.Tensor) -> torch.Tensor:
    """Compute the standard minimum-jerk polynomial interpolation s(u) in [0,1]."""
    return 10.0 * u**3 - 15.0 * u**4 + 6.0 * u**5

def _get_knee_knots_deg(command: torch.Tensor) -> torch.Tensor:
    """Return knee angle knots (in degrees) at key gait events for each env: [N, 8]."""
    v = torch.abs(command[:, 0])
    N = v.shape[0]

    q_OT = (-3.6449 * v * v) + (17.6143 * v) + 6.0109
    q_FA = (0.2867 * v * v) + (3.0904 * v) + 59.6052

    q = [
        torch.full((N,), 7.0, device=v.device),  # t_IC (0%)
        q_OT,  # t_OT (12%)
        torch.full((N,), 10.0, device=v.device),  # t_HR (31%)
        torch.full((N,), 23.0, device=v.device),  # t_OI (50%)
        torch.full((N,), 32.0, device=v.device),  # t_TO (62%)
        q_FA,  # t_FA (73%)
        torch.full((N,), 2.0, device=v.device),  # t_KMF (99%)
        torch.full((N,), 7.0, device=v.device),  # wrap to next t_IC (100%)
    ]
    return torch.stack(q, dim=1)  # [N, 8]

def get_desired_knee_angle(
    env: ManagerBasedEnv,
    period: float,
    offset: list,
    command_name: str = "twist",
    action_name: str | None = None,
) -> torch.Tensor:
    """Compute desired knee angle per leg over gait cycle (radians): [N, 2].

    Uses minimum-jerk spline interpolation over velocity-dependent knee knots,
    with separate phase offsets per leg.

    Args:
        env: The environment instance.
        period: Gait cycle period in seconds.
        offset: Phase offsets for each leg (e.g. [0.0, 0.5]).
        command_name: Name of the velocity command to look up.

    Returns:
        Tensor of shape [N, 2] with desired knee angles in radians.
    """
    if action_name is not None:
        # Use the learnable phase φ_t from GaitFrequencyAction instead of
        # the fixed-period time-based phase.
        gait_term = env.action_manager.get_term(action_name)
        global_phase = gait_term.phase.unsqueeze(1)  # [B, 1]
    else:
        global_phase = ( # before unsqueeze: shape (envs, )
            ((env.episode_length_buf * env.step_dt) % period) / period
        ).unsqueeze(1) # after: shape (envs, 1)

    phases = [(global_phase + off) % 1.0 for off in offset]
    leg_phase = torch.cat(phases, dim=-1)

    t = torch.tensor(
        [0.00, 0.12, 0.31, 0.50, 0.62, 0.73, 0.99, 1.00], device=leg_phase.device
    )
    K = t.numel() - 1

    t0 = t[:-1].view(1, 1, K)  # [1, 1, K]
    t1 = t[1:].view(1, 1, K)  # [1, 1, K]

    command = env.command_manager.get_command(command_name)
    assert command is not None, f"Command '{command_name}' not found."

    q_rad = _get_knee_knots_deg(command) * torch.pi / 180.0
    q0 = q_rad[:, :-1].unsqueeze(1)  # [N, 1, K]
    q1 = q_rad[:, 1:].unsqueeze(1)  # [N, 1, K]

    tau = leg_phase.unsqueeze(-1)  # [N, 2, 1]

    seg_mask = (tau >= t0) & (tau < t1)  # [N, 2, K]

    at_one = tau >= 1.0 - 1e-6  # [N, 2, 1]
    if at_one.any():
        last = torch.zeros_like(seg_mask)
        last[..., -1] = at_one.squeeze(-1)
        empty = ~seg_mask.any(dim=-1, keepdim=True)
        seg_mask = torch.where(empty, last, seg_mask)

    denom = (t1 - t0).clamp_min(1e-6)  # [1, 1, K]
    u = ((tau - t0) / denom).clamp(0.0, 1.0)  # [N, 2, K]
    s = _minjerk(u)  # [N, 2, K]

    y_seg = q0 + (q1 - q0) * s  # [N, 2, K]
    y = (y_seg * seg_mask).sum(dim=-1)  # [N, 2]

    stationary_mask = torch.norm(command[:, :2], dim=1) < 0.1
    if stationary_mask.any():
        stand_theta = q_rad[:, 3]  # [N]
        y[stationary_mask] = stand_theta[stationary_mask].unsqueeze(-1).expand(-1, 2)

    return torch.nan_to_num(y, nan=0.0)  # [N, 2]
