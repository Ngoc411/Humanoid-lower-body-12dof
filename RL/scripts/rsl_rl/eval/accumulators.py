# Copyright (c) 2025-2026, The RoboLab Project Developers.
# SPDX-License-Identifier: BSD-3-Clause
"""Per-step data collection during a rollout.

`EpisodeRecorder` appends named torch tensors each control step and, at the end,
returns numpy arrays shaped (num_envs, T, ...) so the pure metric functions can
run per env. We deliberately keep everything in RAM: metrics are computed on ALL
envs, while only a few representative envs are later written to parquet.

`RobotSampler` is a thin helper that pulls the right quantities out of the live
Isaac Lab env each step (joint pos/vel, torque, CoM, feet, contact, tilt).
"""

from __future__ import annotations

import re

import numpy as np
import torch


class EpisodeRecorder:
    """Collect (T, num_envs, ...) tensors keyed by name."""

    def __init__(self) -> None:
        self._data: dict[str, list[torch.Tensor]] = {}
        self._t = 0

    def append(self, **kwargs: torch.Tensor) -> None:
        for key, val in kwargs.items():
            self._data.setdefault(key, []).append(val.detach().to("cpu"))
        self._t += 1

    @property
    def num_steps(self) -> int:
        return self._t

    def stack(self) -> dict[str, np.ndarray]:
        """Return dict of numpy arrays shaped (num_envs, T, ...)."""
        out: dict[str, np.ndarray] = {}
        for key, seq in self._data.items():
            # stack over time -> (T, N, ...) then move time to axis 1.
            arr = torch.stack(seq, dim=0).numpy()
            out[key] = np.swapaxes(arr, 0, 1)
        return out


class RobotSampler:
    """Reads evaluation quantities from a live ManagerBasedRLEnv each step.

    Resolves body/joint indices once from the unwrapped env so the per-step
    reads are plain tensor indexing.
    """

    def __init__(self, env, cfg) -> None:
        self.cfg = cfg
        u = env.unwrapped
        self.env = u
        self.robot = u.scene[cfg.ROBOT_ASSET]
        self.contact = u.scene.sensors[cfg.CONTACT_SENSOR]
        self.device = u.device

        # joint order actually used by the articulation
        self.joint_names = list(self.robot.data.joint_names)
        self.num_joints = len(self.joint_names)

        # feet bodies on the articulation (for pos/vel) and on the sensor (forces)
        self.left_foot_body = self.robot.find_bodies(cfg.LEFT_FOOT_REGEX)[0]
        self.right_foot_body = self.robot.find_bodies(cfg.RIGHT_FOOT_REGEX)[0]
        # ContactSensor body order can differ from the articulation; match by name.
        sensor_names = list(self.contact.body_names)
        self.left_foot_sensor = [i for i, n in enumerate(sensor_names)
                                 if re.search(cfg.LEFT_FOOT_REGEX, n)]
        self.right_foot_sensor = [i for i, n in enumerate(sensor_names)
                                  if re.search(cfg.RIGHT_FOOT_REGEX, n)]
        self.base_body = self.robot.find_bodies(cfg.BASE_BODY_NAME)[0]
        if not (self.left_foot_sensor and self.right_foot_sensor):
            raise RuntimeError(
                f"Foot bodies not found on contact sensor. body_names={sensor_names}")

        self.up_world = torch.tensor([0.0, 0.0, 1.0], device=self.device)

    # ---- per-step reads -------------------------------------------------- #
    def joint_pos(self) -> torch.Tensor:
        return self.robot.data.joint_pos.clone()

    def joint_vel(self) -> torch.Tensor:
        return self.robot.data.joint_vel.clone()

    def torque(self) -> torch.Tensor:
        # applied_torque is the actual torque sent to the joints (post-actuator).
        return self.robot.data.applied_torque.clone()

    def root_pos_local(self) -> torch.Tensor:
        """Root position with the per-env origin removed (so x,y start near 0)."""
        return (self.robot.data.root_pos_w - self.env.scene.env_origins).clone()

    def root_lin_vel_b(self) -> torch.Tensor:
        return self.robot.data.root_lin_vel_b.clone()

    def root_lin_vel_w(self) -> torch.Tensor:
        return self.robot.data.root_lin_vel_w.clone()

    def root_ang_vel_b(self) -> torch.Tensor:
        return self.robot.data.root_ang_vel_b.clone()

    def base_height(self) -> torch.Tensor:
        return self.robot.data.root_pos_w[:, 2].clone()

    def tilt(self) -> torch.Tensor:
        """Angle (rad) between the torso up-axis and world up = projected_gravity."""
        # projected_gravity_b z-component: -1 upright, 0 horizontal.
        pg = self.robot.data.projected_gravity_b  # (N, 3), unit gravity in base frame
        cos_tilt = (-pg[:, 2]).clamp(-1.0, 1.0)
        return torch.arccos(cos_tilt)

    def foot_pos_w(self) -> torch.Tensor:
        """(N, 2, 3) world positions of [left, right] feet, env-origin removed."""
        idx = self.left_foot_body + self.right_foot_body
        pos = self.robot.data.body_pos_w[:, idx, :] - self.env.scene.env_origins[:, None, :]
        return pos.clone()

    def foot_vel_w(self) -> torch.Tensor:
        idx = self.left_foot_body + self.right_foot_body
        return self.robot.data.body_lin_vel_w[:, idx, :].clone()

    def foot_contact(self) -> torch.Tensor:
        """(N, 2) net contact-force magnitude for [left, right] feet."""
        forces = self.contact.data.net_forces_w  # (N, num_bodies, 3)
        l = forces[:, self.left_foot_sensor, :].norm(dim=-1).max(dim=1).values
        r = forces[:, self.right_foot_sensor, :].norm(dim=-1).max(dim=1).values
        return torch.stack([l, r], dim=-1).clone()

    def command(self) -> torch.Tensor:
        return self.env.command_manager.get_command("base_velocity")[:, :3].clone()

    # ---- bundle ---------------------------------------------------------- #
    def sample(self) -> dict[str, torch.Tensor]:
        """All standard signals for one step (used by every phase)."""
        return {
            "joint_pos": self.joint_pos(),
            "joint_vel": self.joint_vel(),
            "torque": self.torque(),
            "root_pos": self.root_pos_local(),
            "root_lin_vel_b": self.root_lin_vel_b(),
            "root_lin_vel_w": self.root_lin_vel_w(),
            "base_z": self.base_height(),
            "tilt": self.tilt(),
            "foot_pos": self.foot_pos_w(),
            "foot_vel": self.foot_vel_w(),
            "foot_contact": self.foot_contact(),
            "command": self.command(),
        }

    def ref_joint_pos(self, animation_term: str) -> torch.Tensor:
        """Current reference joint pos from the animation manager (window step 0)."""
        term = self.env.animation_manager.get_term(animation_term)
        return term.get_dof_pos()[:, 0, :].clone()

    def ref_joint_vel(self, animation_term: str) -> torch.Tensor:
        term = self.env.animation_manager.get_term(animation_term)
        return term.get_dof_vel()[:, 0, :].clone()
