# Copyright (c) 2022-2025, The Isaac Lab Project Developers.
# Copyright (c) 2025-2026, The RoboLab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause
"""Evaluate a trained AMP policy and write 3 result files.

Phases (each = one process run):
  motion : phase-normalized joint tracking vs the 3 reference clips
  task   : dense per-axis velocity sweep + step-response + energy/survival
  robust : push recovery (set-root-velocity) x cmd_vx x direction + obs-noise sweep

Examples (use the same launcher you run play_amp.py with):
  isaaclab.bat -p scripts/rsl_rl/eval_amp.py --task Chaos-AMP-Play --num_envs 64  --headless --phase motion --checkpoint .../model.pt
  isaaclab.bat -p scripts/rsl_rl/eval_amp.py --task Chaos-AMP-Play --num_envs 256 --headless --phase task   --checkpoint .../model.pt
  isaaclab.bat -p scripts/rsl_rl/eval_amp.py --task Chaos-AMP-Play --num_envs 600 --headless --phase robust --checkpoint .../model.pt
  ... --phase all   runs the three in sequence.
"""

"""Launch Isaac Sim Simulator first."""

import argparse
import sys

from isaaclab.app import AppLauncher

import cli_args  # isort: skip

parser = argparse.ArgumentParser(description="Evaluate an AMP policy (3 phases).")
parser.add_argument("--num_envs", type=int, default=None, help="Number of environments.")
parser.add_argument("--task", type=str, default=None, help="Name of the task.")
parser.add_argument("--agent", type=str, default="rsl_rl_cfg_entry_point", help="RL agent config entry point.")
parser.add_argument("--seed", type=int, default=None, help="Environment seed.")
parser.add_argument("--phase", type=str, default="all",
                    choices=["motion", "task", "robust", "all"], help="Which evaluation phase to run.")
parser.add_argument("--terrain", type=str, default="flat",
                    choices=["flat", "rough"], help="Terrain type: flat (plane) or rough (random uneven).")
parser.add_argument("--eval_steps", type=int, default=None,
                    help="Override rollout length (control steps) for motion/task phases.")
parser.add_argument("--no_plots", action="store_true", default=False, help="Skip PNG generation.")
parser.add_argument("--out_dir", type=str, default=None, help="Override output directory.")
cli_args.add_rsl_rl_args(parser)
AppLauncher.add_app_launcher_args(parser)
args_cli, hydra_args = parser.parse_known_args()

sys.argv = [sys.argv[0]] + hydra_args

app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

"""Rest everything follows."""

import gymnasium as gym
import os
import time

import torch

from rsl_rl.runners import OnPolicyRunner

from isaaclab.envs import DirectMARLEnv, DirectRLEnvCfg, DirectMARLEnvCfg, ManagerBasedRLEnvCfg, multi_agent_to_single_agent
from isaaclab.utils.assets import retrieve_file_path

from isaaclab_rl.rsl_rl import RslRlBaseRunnerCfg, RslRlVecEnvWrapper

import isaaclab_tasks  # noqa: F401
from isaaclab_tasks.utils import get_checkpoint_path
from isaaclab_tasks.utils.hydra import hydra_task_config

import robolab.tasks  # noqa: F401
from robolab import ROBOLAB_ROOT_DIR

import isaaclab.terrains as terrain_gen

from eval import config as C
from eval import phases as P
from eval import writers as W
from eval import plotting


def _configure_terrain(env_cfg, terrain: str) -> None:
    """Override terrain in env_cfg before env is created.

    flat  → simple infinite plane (no generator overhead, most stable for metrics)
    rough → random uneven terrain (noise 2-6cm) to test generalization
    """
    t = env_cfg.scene.terrain
    if terrain == "flat":
        t.terrain_type = "plane"
        t.terrain_generator = None
    elif terrain == "rough":
        t.terrain_type = "generator"
        t.terrain_generator = terrain_gen.TerrainGeneratorCfg(
            size=(8.0, 8.0),
            border_width=2.0,
            num_rows=4,
            num_cols=4,
            horizontal_scale=0.05,
            vertical_scale=0.005,
            slope_threshold=0.75,
            sub_terrains={
                "random_rough": terrain_gen.HfRandomUniformTerrainCfg(
                    proportion=1.0,
                    noise_range=(0.02, 0.06),   # 2-6 cm bump height
                    noise_step=0.005,
                    border_width=0.25,
                ),
            },
        )


def _disable_eval_resets(env_cfg):
    """Keep envs alive (we detect falls ourselves) and freeze commands."""
    # commands: no auto-resample, no heading controller, no random standing envs
    bv = env_cfg.commands.base_velocity
    bv.heading_command = False
    bv.resampling_time_range = (1.0e9, 1.0e9)
    bv.rel_standing_envs = 0.0
    bv.rel_heading_envs = 0.0
    # terminations: only time_out, so stumbles don't reset mid-rollout
    if hasattr(env_cfg.terminations, "base_contact"):
        env_cfg.terminations.base_contact = None
    if hasattr(env_cfg.terminations, "bad_orientation"):
        env_cfg.terminations.bad_orientation = None
    # remove training push so our manual push is the only perturbation
    if hasattr(env_cfg.events, "push_robot"):
        env_cfg.events.push_robot = None


def _robot_mass(env):
    robot = env.unwrapped.scene[C.ROBOT_ASSET]
    try:
        return float(robot.root_physx_view.get_masses()[0].sum())
    except Exception:
        return float(getattr(robot.data, "default_mass", torch.tensor([[1.0]]))[0].sum())


@hydra_task_config(args_cli.task, args_cli.agent)
def main(env_cfg: ManagerBasedRLEnvCfg | DirectRLEnvCfg | DirectMARLEnvCfg, agent_cfg: RslRlBaseRunnerCfg):
    agent_cfg = cli_args.update_rsl_rl_cfg(agent_cfg, args_cli)
    env_cfg.scene.num_envs = args_cli.num_envs if args_cli.num_envs is not None else env_cfg.scene.num_envs
    env_cfg.seed = agent_cfg.seed
    env_cfg.sim.device = args_cli.device if args_cli.device is not None else env_cfg.sim.device

    _configure_terrain(env_cfg, args_cli.terrain)
    _disable_eval_resets(env_cfg)

    # locate checkpoint
    log_root_path = os.path.abspath(os.path.join("logs", "rsl_rl", agent_cfg.experiment_name))
    if args_cli.checkpoint:
        resume_path = retrieve_file_path(args_cli.checkpoint)
    else:
        resume_path = get_checkpoint_path(log_root_path, agent_cfg.load_run, agent_cfg.load_checkpoint)
    log_dir = os.path.dirname(resume_path)
    env_cfg.log_dir = log_dir

    # build env
    env = gym.make(args_cli.task, cfg=env_cfg, render_mode=None)
    if isinstance(env.unwrapped, DirectMARLEnv):
        env = multi_agent_to_single_agent(env)
    env = RslRlVecEnvWrapper(env, clip_actions=agent_cfg.clip_actions)

    # load policy
    if agent_cfg.class_name == "AMPRunner":
        from rsl_rl.runners import AMPRunner
        runner = AMPRunner(env, agent_cfg.to_dict(), log_dir=None, device=agent_cfg.device)
    else:
        runner = OnPolicyRunner(env, agent_cfg.to_dict(), log_dir=None, device=agent_cfg.device)
    runner.load(resume_path, map_location=agent_cfg.device)
    policy = runner.get_inference_policy(device=env.unwrapped.device)
    try:
        policy_nn = runner.alg.policy
    except AttributeError:
        policy_nn = runner.alg.actor_critic

    fps = 1.0 / env.unwrapped.step_dt
    mass = _robot_mass(env)
    motions_dir = os.path.join(ROBOLAB_ROOT_DIR, "data", "motions")
    default_steps = int(env.unwrapped.max_episode_length)
    n_steps = args_cli.eval_steps or min(default_steps, int(15.0 * fps))

    # output directory — terrain type is included in the folder name
    stamp = time.strftime("%Y%m%d_%H%M%S")
    folder = f"{stamp}_{args_cli.terrain}"
    out_dir = args_cli.out_dir or os.path.join("logs", "eval", agent_cfg.experiment_name, folder)
    out_dir = os.path.abspath(out_dir)
    ts_dir = os.path.join(out_dir, "timeseries")
    print(f"[eval] checkpoint = {resume_path}")
    print(f"[eval] terrain={args_cli.terrain}  fps={fps:.1f}  mass={mass:.2f}kg  n_steps={n_steps}  out={out_dir}")

    phase = args_cli.phase
    terrain_tag = args_cli.terrain
    motion_json = task_json = robust_json = None

    if phase in ("motion", "all"):
        print("\n==== PHASE 2: motion quality ====")
        motion_json, (s, e, c) = P.run_motion_phase(env, policy, policy_nn, fps, mass, motions_dir, n_steps)
        motion_json["meta"]["terrain"] = terrain_tag
        W.save_json(os.path.join(out_dir, "2_motion_quality.json"), motion_json)
        W.write_timeseries_parquet(os.path.join(ts_dir, "motion_quality.parquet"),
                                   W.stack_series(s), e, c, fps)

    if phase in ("task", "all"):
        print("\n==== PHASE 3: task metrics ====")
        task_json, (s, e, c), (srs, sre, src) = P.run_task_phase(env, policy, policy_nn, fps, mass, n_steps)
        task_json["meta"]["terrain"] = terrain_tag
        W.save_json(os.path.join(out_dir, "3_task_metrics.json"), task_json)
        W.write_timeseries_parquet(os.path.join(ts_dir, "velocity_sweep.parquet"),
                                   W.stack_series(s), e, c, fps)
        if srs:
            W.write_timeseries_parquet(os.path.join(ts_dir, "velocity_step_response.parquet"),
                                       srs, sre, src, fps)

    if phase in ("robust", "all"):
        print("\n==== PHASE 4: robustness ====")
        robust_json, (s, e, c) = P.run_robust_phase(env, policy, policy_nn, fps, mass)
        robust_json["meta"]["terrain"] = terrain_tag
        W.save_json(os.path.join(out_dir, "4_robustness.json"), robust_json)
        W.write_timeseries_parquet(os.path.join(ts_dir, "push_recovery.parquet"), s, e, c, fps)

    if not args_cli.no_plots:
        plotting.plot_all(os.path.join(out_dir, "plots"),
                          motion=motion_json, task=task_json, robust=robust_json)

    print(f"\n[eval] done -> {out_dir}")
    env.close()


if __name__ == "__main__":
    main()
    simulation_app.close()
