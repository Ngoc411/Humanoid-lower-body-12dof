# Copyright (c) 2025-2026, The RoboLab Project Developers.
# SPDX-License-Identifier: BSD-3-Clause

# IMPORTANT: import tensordict BEFORE AppLauncher — same reason as play.py
import tensordict  # noqa: F401  # isort: skip

import argparse
import sys
import os
import time

from isaaclab.app import AppLauncher

# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------
parser = argparse.ArgumentParser(description="Replay a PKL motion file on the Chaos robot in Isaac Sim.")
parser.add_argument("--pkl", type=str,
    default=os.path.join(
        os.path.dirname(__file__), "..", "..", "data", "motions",
        "walk1_forward_retargeted_ik_01.pkl"
    ),
    help="Path to the .pkl motion file (30 fps)")
parser.add_argument("--task", type=str, default="Chaos-AMP-Play",
    help="Isaac Lab registered task name")
parser.add_argument("--num_envs", type=int, default=1)
parser.add_argument("--kinematic", action="store_true",
    help="Kinematic mode: write joint state directly each step (no physics lag). "
         "Default: PD-tracking mode (physics runs, may show tracking error / fall).")
parser.add_argument("--loop", action="store_true",
    help="Loop the motion clip forever")
parser.add_argument("--speed", type=float, default=100.0,
    help="Playback speed in percent (100 = normal, 150 = 1.5x faster, 50 = half speed)")
parser.add_argument("--no_interp", action="store_true",
    help="Apply PKL frames directly (no interpolation). Each 30fps frame is held for "
         "the natural number of 50Hz policy steps it spans — no blending between frames.")
parser.add_argument("--plane", action="store_true",
    help="Force flat-plane terrain (overrides task terrain config)")
AppLauncher.add_app_launcher_args(parser)
args_cli, hydra_args = parser.parse_known_args()
sys.argv = [sys.argv[0]] + hydra_args

app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

# ---------------------------------------------------------------------------
# Post-launch imports
# ---------------------------------------------------------------------------
import torch
import joblib
import numpy as np
import gymnasium as gym

import isaaclab_tasks  # noqa: F401
import robolab.tasks  # noqa: F401

from isaaclab.envs import ManagerBasedRLEnvCfg
from isaaclab_rl.rsl_rl import RslRlVecEnvWrapper
from isaaclab_tasks.utils.hydra import hydra_task_config

# ---------------------------------------------------------------------------
# Joint order in the PKL (from atom01_amp_env_cfg.py DATASET_JOINT_ORDER)
# ---------------------------------------------------------------------------
DATASET_JOINT_ORDER = [
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
    'right_ankle_roll_joint',
]


# ---------------------------------------------------------------------------
# Motion helpers
# ---------------------------------------------------------------------------

def load_pkl(path: str) -> dict:
    path = os.path.normpath(os.path.abspath(path))
    if not os.path.exists(path):
        raise FileNotFoundError(f"PKL not found: {path}")
    data = joblib.load(path)
    if not isinstance(data, dict):
        raise ValueError(f"PKL must be a dict, got {type(data)}")
    for k in ("fps", "root_pos", "root_rot", "dof_pos"):
        if k not in data:
            raise KeyError(f"Missing key '{k}' in PKL")
    fps = float(data["fps"])
    n = len(data["root_pos"])
    dur = (n - 1) / fps
    print(f"[MotionReplay] Loaded: {path}")
    print(f"[MotionReplay]   fps={fps:.1f} | frames={n} | duration={dur:.2f}s")
    print(f"[MotionReplay]   dof_pos shape: {np.array(data['dof_pos']).shape}")
    # Pre-convert to tensors on CPU; will move to device later
    data["_dof_pos_t"] = torch.tensor(np.array(data["dof_pos"]), dtype=torch.float32)
    data["_root_pos_t"] = torch.tensor(np.array(data["root_pos"]), dtype=torch.float32)
    data["_root_rot_t"] = torch.tensor(np.array(data["root_rot"]), dtype=torch.float32)
    data["_fps"] = fps
    data["_n"] = n
    data["_dur"] = dur
    return data


def build_remap(sim_joint_names: list[str], dataset_order: list[str]) -> list[int]:
    """
    For each joint in sim_joint_names, return its index in dataset_order.
    Returns -1 if not found (will keep default position for that joint).
    """
    remap = []
    for name in sim_joint_names:
        if name in dataset_order:
            remap.append(dataset_order.index(name))
        else:
            print(f"[MotionReplay] WARNING: '{name}' not in dataset — will use default pos")
            remap.append(-1)
    return remap


def apply_remap(dataset_dof: torch.Tensor, remap: list[int], default_pos: torch.Tensor, device: str) -> torch.Tensor:
    """Map dataset DOF order → sim joint order. Missing joints use default_pos."""
    out = default_pos.clone().to(device)
    for sim_i, ds_i in enumerate(remap):
        if ds_i >= 0:
            out[sim_i] = dataset_dof[ds_i]
    return out


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

@hydra_task_config(args_cli.task, "rsl_rl_cfg_entry_point")
def main(env_cfg: ManagerBasedRLEnvCfg, agent_cfg):  # noqa: ANN001

    # ---- env config patches ----
    env_cfg.scene.num_envs = args_cli.num_envs
    if hasattr(args_cli, "device") and args_cli.device:
        env_cfg.sim.device = args_cli.device

    if args_cli.plane:
        import isaaclab.sim as sim_utils_mod
        env_cfg.scene.terrain.terrain_type = "plane"
        env_cfg.scene.terrain.terrain_generator = None

    # Long episode so env doesn't auto-reset while watching
    env_cfg.episode_length_s = 9999.0

    # Camera in front of the robot, looking at waist height
    from isaaclab.envs import ViewerCfg
    env_cfg.viewer = ViewerCfg(
        eye=(3.0, 0.0, 0.8),   # 2m in front, 0.8m high
        lookat=(0.0, 0.0, 0.2),
        origin_type="asset_root",
        env_index=0,
        asset_name="robot",
    )

    # ---- load PKL before creating env (fast, CPU only) ----
    pkl_data = load_pkl(args_cli.pkl)
    dur = pkl_data["_dur"]

    # ---- create env ----
    env_raw = gym.make(args_cli.task, cfg=env_cfg, render_mode=None)
    env = RslRlVecEnvWrapper(env_raw, clip_actions=agent_cfg.clip_actions)

    device = str(env.unwrapped.device)
    robot = env.unwrapped.scene["robot"]

    # ---- joint mapping ----
    sim_joint_names = list(robot.data.joint_names)
    num_joints = len(sim_joint_names)
    print(f"[MotionReplay] Sim joint names ({num_joints}): {sim_joint_names}")

    remap = build_remap(sim_joint_names, DATASET_JOINT_ORDER)

    # Default joint positions for action conversion and fallback
    default_dof = robot.data.default_joint_pos[0].clone().to(device)  # (num_joints,)
    action_scale = 0.25  # must match ActionsCfg.joint_pos.scale

    # ---- diagnostic: verify mapping and detect unit issues ----
    frame0_pkl = pkl_data["_dof_pos_t"][0]  # (num_pkl_joints,) — frame 0 raw values
    max_abs = float(frame0_pkl.abs().max().item())
    unit_warn = " *** VALUES LOOK LIKE DEGREES — PKL may need /180*pi? ***" if max_abs > 6.3 else ""
    print(f"\n[MotionReplay] === JOINT MAPPING DIAGNOSTIC ===")
    print(f"  PKL frame-0 max|val|={max_abs:.3f} rad  (>6.3 ⇒ likely degrees){unit_warn}")
    print(f"  {'sim_idx':>7}  {'sim_joint_name':<35}  {'pkl_idx':>7}  {'pkl_val_rad':>11}  {'pkl_val_deg':>11}  {'sim_default_rad':>15}")
    for i, name in enumerate(sim_joint_names):
        ds_i = remap[i]
        pkl_val = float(frame0_pkl[ds_i].item()) if ds_i >= 0 else float("nan")
        sim_def = float(default_dof[i].item())
        print(f"  {i:>7}  {name:<35}  {ds_i:>7}  {pkl_val:>+11.4f}  {pkl_val*180/3.14159:>+10.2f}°  {sim_def:>+15.4f}")
    print(f"[MotionReplay] ==========================================\n")

    dt = env.unwrapped.step_dt  # 0.02 s @ 50 Hz
    policy_hz = 1.0 / dt
    speed_mul = args_cli.speed / 100.0  # convert percent → multiplier

    interp_mode = "DISCRETE (no interp, native 30fps frames)" if args_cli.no_interp else "INTERPOLATED (lerp to 50Hz)"
    print(f"\n[MotionReplay] Drive   : {'KINEMATIC' if args_cli.kinematic else 'PD-TRACKING'}")
    print(f"[MotionReplay] Frames  : {interp_mode}")
    print(f"[MotionReplay] PKL fps : {pkl_data['_fps']:.1f}  →  sim Hz : {policy_hz:.0f}")
    print(f"[MotionReplay] Speed   : {args_cli.speed:.0f}%  ({speed_mul:.2f}x)  |  Loop: {args_cli.loop}")
    print(f"[MotionReplay] Duration: {dur:.2f}s\n")

    # ---- frame accumulator (no float drift) ----
    # Instead of accumulating motion_time in seconds, track frame_idx (int) +
    # frame_acc (float remainder within current frame).
    # Each policy step: frame_acc += dt * speed_mul.
    # When frame_acc >= frame_dt: advance frame_idx, subtract frame_dt.
    # blend = frame_acc / frame_dt gives lerp weight always in [0, 1).
    fps_pkl = pkl_data["_fps"]
    n_frames = pkl_data["_n"]
    frame_dt = 1.0 / fps_pkl        # PKL frame duration (1/30 ≈ 0.0333s)
    frame_idx = 0
    frame_acc = 0.0                  # elapsed time within current PKL frame
    motion_ended = False

    # ---- sim loop ----
    obs = env.get_observations()
    timestep = 0
    wall_start = time.time()

    while simulation_app.is_running():
        with torch.inference_mode():

            # --- compute target DOF from current frame ---
            dof_t = pkl_data["_dof_pos_t"]
            if args_cli.no_interp:
                # Hold exact PKL frame — no blending
                raw_dof = dof_t[frame_idx].to(device)
            else:
                # Lerp between frame_idx and frame_idx+1 using sub-frame position
                blend = frame_acc / frame_dt
                f0 = frame_idx
                f1 = min(frame_idx + 1, n_frames - 1)
                raw_dof = torch.lerp(dof_t[f0].to(device), dof_t[f1].to(device), blend)

            target_dof = apply_remap(raw_dof, remap, default_dof, device)
            target_dof_batch = target_dof.unsqueeze(0).expand(args_cli.num_envs, -1)

            # action for PD controller: (target - default) / scale
            # Used in both modes so PD doesn't fight the written state
            action = (target_dof_batch - default_dof.unsqueeze(0)) / action_scale
            action = torch.clamp(action, -1.0, 1.0)

            if args_cli.kinematic:
                # Write positions directly AND pass matching action so PD target == written pos
                dof_vel_batch = torch.zeros_like(target_dof_batch)
                robot.write_joint_state_to_sim(target_dof_batch, dof_vel_batch)

            obs, _, terminated, _ = env.step(action)

            # --- advance frame accumulator ---
            if not motion_ended:
                frame_acc += dt * speed_mul
                if frame_acc >= frame_dt:
                    frame_acc -= frame_dt
                    frame_idx += 1
                    if frame_idx >= n_frames:
                        if args_cli.loop:
                            frame_idx = 0
                            frame_acc = 0.0
                            print(f"[MotionReplay] step={timestep}  loop restart")
                        else:
                            frame_idx = n_frames - 1
                            frame_acc = 0.0
                            motion_ended = True
                            print(f"[MotionReplay] step={timestep}  motion ended — holding last pose")

            # env terminated → reset, but keep frame position
            if terminated.any():
                print(f"[MotionReplay] step={timestep}  env reset (terminated)")
                obs = env.get_observations()

            timestep += 1

            # --- periodic status ---
            if timestep % 100 == 0:
                elapsed = time.time() - wall_start
                sim_dof = robot.data.joint_pos[0]
                err = float(torch.mean(torch.abs(sim_dof - target_dof)).item())
                t_motion = frame_idx / fps_pkl + frame_acc
                print(
                    f"[RTF] step={timestep:5d}"
                    f" | frame={frame_idx:4d}/{n_frames} (t={t_motion:.2f}s)"
                    f" | SPS={timestep/elapsed:5.1f}"
                    f" | RTF={timestep*dt/elapsed:.2f}x"
                    f" | joint_err={err:.4f} rad"
                )

    env.close()


if __name__ == "__main__":
    main()
    simulation_app.close()
