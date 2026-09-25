# Copyright (c) 2022-2026, The Isaac Lab Project Developers (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Script to play a checkpoint if an RL agent from RSL-RL."""

"""Launch Isaac Sim Simulator first."""

import argparse
import sys
import weakref

# IMPORTANT (Windows): import tensordict's native extension BEFORE any isaaclab /
# carb / omni import. rsl-rl-lib 3.x depends on tensordict, whose compiled `_C.pyd`
# can crash with a fatal access violation if it loads AFTER Omniverse Kit injects
# its runtime/torch DLLs into the process. This mirrors the train.py workaround.
import tensordict  # noqa: F401

from isaaclab.app import AppLauncher

# local imports
import cli_args  # isort: skip

# add argparse arguments
parser = argparse.ArgumentParser(description="Train an RL agent with RSL-RL.")
parser.add_argument("--video", action="store_true", default=False, help="Record videos during training.")
parser.add_argument("--video_length", type=int, default=200, help="Length of the recorded video (in steps).")
parser.add_argument(
    "--disable_fabric", action="store_true", default=False, help="Disable fabric and use USD I/O operations."
)
parser.add_argument("--num_envs", type=int, default=None, help="Number of environments to simulate.")
parser.add_argument("--task", type=str, default=None, help="Name of the task.")
parser.add_argument(
    "--agent", type=str, default="rsl_rl_cfg_entry_point", help="Name of the RL agent configuration entry point."
)
parser.add_argument("--seed", type=int, default=None, help="Seed used for the environment")
parser.add_argument(
    "--use_pretrained_checkpoint",
    action="store_true",
    help="Use the pre-trained checkpoint from Nucleus.",
)
parser.add_argument("--print_policy_data", action="store_true", help="Print policy obs/actions at each policy step.")
parser.add_argument(
    "--latest_frame",
    action="store_true",
    help="With --print_policy_data, print only the newest history frame of each obs term (per-term labelled).",
)
parser.add_argument("--real-time", action="store_true", default=False, help="Run in real-time, if possible.")
# append RSL-RL cli arguments
cli_args.add_rsl_rl_args(parser)
# append AppLauncher cli args
AppLauncher.add_app_launcher_args(parser)
# parse the arguments
args_cli, hydra_args = parser.parse_known_args()
# always enable cameras to record video
if args_cli.video:
    args_cli.enable_cameras = True

# clear out sys.argv for Hydra
sys.argv = [sys.argv[0]] + hydra_args

# launch omniverse app
app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

"""Check for installed RSL-RL version."""

import importlib.metadata as metadata

from packaging import version

installed_version = metadata.version("rsl-rl-lib")

"""Rest everything follows."""

import os
import time
from collections.abc import Callable

import gymnasium as gym
import torch
from rsl_rl.runners import DistillationRunner, OnPolicyRunner

from isaaclab.envs import (
    DirectMARLEnv,
    DirectMARLEnvCfg,
    DirectRLEnvCfg,
    ManagerBasedRLEnvCfg,
    multi_agent_to_single_agent,
)
from isaaclab.utils.assets import retrieve_file_path
from isaaclab.utils.dict import print_dict

from isaaclab_rl.rsl_rl import (
    RslRlBaseRunnerCfg,
    RslRlVecEnvWrapper,
    export_policy_as_jit,
    export_policy_as_onnx,
    handle_deprecated_rsl_rl_cfg,
    handle_deprecated_rsl_rl_checkpoint,
)
from isaaclab_rl.utils.pretrained_checkpoint import get_published_pretrained_checkpoint

import isaaclab_tasks  # noqa: F401
from isaaclab_tasks.utils import get_checkpoint_path
from isaaclab_tasks.utils.hydra import hydra_task_config

import carb
import omni

import Vin_Rsl_Rl.tasks  # noqa: F401


class Keyboard:
    """RoboLab-style keyboard controller for velocity command playback."""

    def __init__(
        self,
        env: RslRlVecEnvWrapper,
        command_name: str = "twist",
        lin_vel_step: float = 0.05,
        ang_vel_step: float = 0.05,
        push_speed: float = 0.5,
    ):
        self.env = env
        self.command_name = command_name
        self.lin_vel_step = lin_vel_step
        self.ang_vel_step = ang_vel_step
        self.push_speed = push_speed

        self.lin_vel_x = 0.0
        self.lin_vel_y = 0.0
        self.ang_vel = 0.0

        self._appwindow = omni.appwindow.get_default_app_window()
        self._input = carb.input.acquire_input_interface()
        self._keyboard = self._appwindow.get_keyboard()
        self._keyboard_sub = self._input.subscribe_to_keyboard_events(
            self._keyboard,
            lambda event, *args, obj=weakref.proxy(self): obj._on_keyboard_event(event, *args),
        )
        self._additional_callbacks: dict[str, Callable] = {}
        self._create_key_bindings()
        self._read_current_command()
        self._update_commands(print_update=True)

        print("[Keyboard] Velocity control initialized:")
        print(f"  W/S : Forward/Backward  (lin_vel_x +/-{lin_vel_step:.2f} m/s)")
        print(f"  Q/E : Turn left/right   (ang_vel   +/-{ang_vel_step:.2f} rad/s)")
        print(f"  A/D : Strafe left/right (lin_vel_y +/-{lin_vel_step:.2f} m/s)")
        print("  X   : Stop all velocities")
        print("  R   : Reset environment")
        print(f"  PUSH (balance test, impulse {push_speed:.2f} m/s):")
        print("  I/K : push forward/back | J/L : push left/right")

    def __del__(self):
        if getattr(self, "_keyboard_sub", None) is not None:
            self._input.unsubscribe_to_keyboard_events(self._keyboard, self._keyboard_sub)
            self._keyboard_sub = None

    def reset(self):
        self.lin_vel_x = 0.0
        self.lin_vel_y = 0.0
        self.ang_vel = 0.0
        self._update_commands(print_update=True)

    def add_callback(self, key: str, func: Callable):
        self._additional_callbacks[key] = func

    def advance(self):
        self._update_commands()

    def _on_keyboard_event(self, event, *args, **kwargs):
        if event.type in (carb.input.KeyboardEventType.KEY_PRESS, carb.input.KeyboardEventType.KEY_REPEAT):
            key = event.input.name
            if key not in self._INPUT_KEY_MAPPING:
                return True

            if key == "R":
                self.env.unwrapped.episode_length_buf[:] = self.env.unwrapped.max_episode_length
                print("[Keyboard] Environment reset triggered")
            elif key == "W":
                self.lin_vel_x += self.lin_vel_step
                self._update_commands(print_update=True)
            elif key == "S":
                self.lin_vel_x -= self.lin_vel_step
                self._update_commands(print_update=True)
            elif key == "Q":
                self.ang_vel += self.ang_vel_step
                self._update_commands(print_update=True)
            elif key == "E":
                self.ang_vel -= self.ang_vel_step
                self._update_commands(print_update=True)
            elif key == "A":
                self.lin_vel_y += self.lin_vel_step
                self._update_commands(print_update=True)
            elif key == "D":
                self.lin_vel_y -= self.lin_vel_step
                self._update_commands(print_update=True)
            elif key == "X":
                self.reset()
                print("[Keyboard] Stopped - all velocities set to zero")
            elif key in ("I", "K", "J", "L"):
                self._apply_push(key)

            if key in self._additional_callbacks:
                self._additional_callbacks[key]()
        return True

    def _apply_push(self, key: str):
        env = self.env.unwrapped
        robot = env.scene["robot"]
        dvx = {"I": self.push_speed, "K": -self.push_speed}.get(key, 0.0)
        dvy = {"J": self.push_speed, "L": -self.push_speed}.get(key, 0.0)
        root_vel = robot.data.root_vel_w.clone()
        root_vel[:, 0] += dvx
        root_vel[:, 1] += dvy
        robot.write_root_velocity_to_sim(root_vel)
        print(f"[Keyboard] PUSH dvx={dvx:+.2f} dvy={dvy:+.2f} m/s")

    def _read_current_command(self):
        env = self.env.unwrapped
        if not hasattr(env, "command_manager") or self.command_name not in env.command_manager.active_terms:
            return
        cmd = env.command_manager.get_command(self.command_name)
        if cmd.numel() == 0:
            return
        self.lin_vel_x = float(cmd[0, 0].item())
        self.lin_vel_y = float(cmd[0, 1].item())
        self.ang_vel = float(cmd[0, 2].item())

    def _update_commands(self, print_update: bool = False):
        env = self.env.unwrapped
        if not hasattr(env, "command_manager") or self.command_name not in env.command_manager.active_terms:
            return

        cmd = env.command_manager.get_command(self.command_name)
        cmd[:, 0] = self.lin_vel_x
        cmd[:, 1] = self.lin_vel_y
        cmd[:, 2] = self.ang_vel

        term = env.command_manager.get_term(self.command_name)
        if hasattr(term, "cfg") and hasattr(term.cfg, "ranges"):
            term.cfg.ranges.lin_vel_x = (self.lin_vel_x, self.lin_vel_x)
            term.cfg.ranges.lin_vel_y = (self.lin_vel_y, self.lin_vel_y)
            term.cfg.ranges.ang_vel_z = (self.ang_vel, self.ang_vel)
            term.cfg.rel_standing_envs = 0.0
        if hasattr(term, "time_left"):
            term.time_left[:] = 1.0e6
        if hasattr(term, "is_standing_env"):
            term.is_standing_env[:] = False
        if hasattr(term, "vel_command_b"):
            term.vel_command_b[:] = cmd

        if print_update:
            print(f"[Keyboard] Vel: vx={self.lin_vel_x:.2f}, vy={self.lin_vel_y:.2f}, ang={self.ang_vel:.2f}")

    def _create_key_bindings(self):
        self._INPUT_KEY_MAPPING = {
            "W": "forward",
            "S": "backward",
            "Q": "turn_left",
            "E": "turn_right",
            "A": "strafe_left",
            "D": "strafe_right",
            "X": "stop",
            "R": "reset_envs",
            "I": "push_forward",
            "K": "push_back",
            "J": "push_left",
            "L": "push_right",
        }


def _to_printable_policy_data(data):
    if isinstance(data, torch.Tensor):
        return data.detach().cpu().tolist()
    if hasattr(data, "items"):
        return {key: _to_printable_policy_data(value) for key, value in data.items()}
    if isinstance(data, (list, tuple)):
        return type(data)(_to_printable_policy_data(value) for value in data)
    return data


def _scale_policy_action(data, scale):
    if isinstance(data, torch.Tensor):
        return data * scale
    if hasattr(data, "items"):
        return {key: _scale_policy_action(value, scale) for key, value in data.items()}
    if isinstance(data, (list, tuple)):
        return type(data)(_scale_policy_action(value, scale) for value in data)
    return data


def _get_obs_group(obs, group_name):
    if hasattr(obs, "get"):
        group_obs = obs.get(group_name, None)
        if group_obs is not None:
            return group_obs
    if hasattr(obs, "__getitem__"):
        try:
            return obs[group_name]
        except (KeyError, TypeError, IndexError):
            pass
    return obs


def _first_n_obs_elements(obs, count):
    if isinstance(obs, torch.Tensor):
        return obs[..., :count]
    return obs


def _split_latest_frame(env, group_name, group_obs):
    """Split a concatenated, history-flattened obs group into only its newest frame, per term.

    Each obs term with history length H and base dim D is stored as H consecutive D-sized
    blocks ordered oldest->newest (IsaacLab ``CircularBuffer.buffer`` keeps the most recent
    entry at the end). With ``flatten_history_dim=True`` the term occupies H*D contiguous
    columns, so the newest frame is the final D columns of that term. Terms are concatenated
    in declaration order, so we walk the vector term-by-term and keep each term's last D
    values, returning an ordered ``{term_name: latest_frame_tensor}`` dict.

    Returns ``None`` (caller falls back to the full obs) if the manager metadata is
    unavailable or a term does not use a simple flattened-history layout.
    """
    if not isinstance(group_obs, torch.Tensor):
        return None
    try:
        obs_manager = env.unwrapped.observation_manager
        term_names = obs_manager.active_terms[group_name]
        term_dims = obs_manager.group_obs_term_dim[group_name]
        term_cfgs = obs_manager._group_obs_term_cfgs[group_name]
    except (AttributeError, KeyError):
        return None

    latest = {}
    idx = 0
    for name, shape, term_cfg in zip(term_names, term_dims, term_cfgs):
        # Only a 1-D (flattened) term has a contiguous last-D "newest frame" slice.
        if len(shape) != 1:
            return None
        flat = int(shape[0])
        hist = term_cfg.history_length if term_cfg.history_length and term_cfg.history_length > 0 else 1
        base = flat // hist
        latest[name] = group_obs[..., idx + flat - base: idx + flat]
        idx += flat
    return latest


@hydra_task_config(args_cli.task, args_cli.agent)
def main(env_cfg: ManagerBasedRLEnvCfg | DirectRLEnvCfg | DirectMARLEnvCfg, agent_cfg: RslRlBaseRunnerCfg):
    """Play with RSL-RL agent."""
    # grab task name for checkpoint path
    task_name = args_cli.task.split(":")[-1]
    train_task_name = task_name.replace("-Play", "")

    # override configurations with non-hydra CLI arguments
    agent_cfg: RslRlBaseRunnerCfg = cli_args.update_rsl_rl_cfg(agent_cfg, args_cli)
    env_cfg.scene.num_envs = args_cli.num_envs if args_cli.num_envs is not None else env_cfg.scene.num_envs

    # handle deprecated configurations
    agent_cfg = handle_deprecated_rsl_rl_cfg(agent_cfg, installed_version)

    # set the environment seed
    # note: certain randomizations occur in the environment initialization so we set the seed here
    env_cfg.seed = agent_cfg.seed
    env_cfg.sim.device = args_cli.device if args_cli.device is not None else env_cfg.sim.device

    # specify directory for logging experiments
    log_root_path = os.path.join("logs", "rsl_rl", agent_cfg.experiment_name)
    log_root_path = os.path.abspath(log_root_path)
    print(f"[INFO] Loading experiment from directory: {log_root_path}")
    if args_cli.use_pretrained_checkpoint:
        resume_path = get_published_pretrained_checkpoint("rsl_rl", train_task_name)
        if not resume_path:
            print("[INFO] Unfortunately a pre-trained checkpoint is currently unavailable for this task.")
            return
    elif args_cli.checkpoint:
        resume_path = retrieve_file_path(args_cli.checkpoint)
    else:
        resume_path = get_checkpoint_path(log_root_path, agent_cfg.load_run, agent_cfg.load_checkpoint)

    log_dir = os.path.dirname(resume_path)

    # set the log directory for the environment (works for all environment types)
    env_cfg.log_dir = log_dir

    # create isaac environment
    env = gym.make(args_cli.task, cfg=env_cfg, render_mode="rgb_array" if args_cli.video else None)

    # convert to single-agent instance if required by the RL algorithm
    if isinstance(env.unwrapped, DirectMARLEnv):
        env = multi_agent_to_single_agent(env)

    # wrap for video recording
    if args_cli.video:
        video_kwargs = {
            "video_folder": os.path.join(log_dir, "videos", "play"),
            "step_trigger": lambda step: step == 0,
            "video_length": args_cli.video_length,
            "disable_logger": True,
        }
        print("[INFO] Recording videos during training.")
        print_dict(video_kwargs, nesting=4)
        env = gym.wrappers.RecordVideo(env, **video_kwargs)

    # wrap around environment for rsl-rl
    env = RslRlVecEnvWrapper(env, clip_actions=agent_cfg.clip_actions)
    keyboard = Keyboard(env) if not args_cli.headless else None  # noqa: F841

    print(f"[INFO]: Loading model checkpoint from: {resume_path}")
    # load previously trained model
    if agent_cfg.class_name == "OnPolicyRunner":
        runner = OnPolicyRunner(env, agent_cfg.to_dict(), log_dir=None, device=agent_cfg.device)
    elif agent_cfg.class_name == "DistillationRunner":
        runner = DistillationRunner(env, agent_cfg.to_dict(), log_dir=None, device=agent_cfg.device)
    else:
        raise ValueError(f"Unsupported runner class: {agent_cfg.class_name}")
    # convert pre-5.0 published checkpoints to the layout expected by rsl-rl >= 5.0 (no-op otherwise)
    resume_path = handle_deprecated_rsl_rl_checkpoint(resume_path, installed_version)
    runner.load(resume_path)

    # obtain the trained policy for inference
    policy = runner.get_inference_policy(device=env.unwrapped.device)

    # export the trained policy to JIT and ONNX formats
    export_model_dir = os.path.join(os.path.dirname(resume_path), "exported")

    if version.parse(installed_version) >= version.parse("4.0.0"):
        # use the new export functions for rsl-rl >= 4.0.0
        runner.export_policy_to_jit(path=export_model_dir, filename="policy.pt")
        runner.export_policy_to_onnx(path=export_model_dir, filename="policy.onnx")
    else:
        # extract the neural network for rsl-rl < 4.0.0
        if version.parse(installed_version) >= version.parse("2.3.0"):
            policy_nn = runner.alg.policy
        else:
            policy_nn = runner.alg.actor_critic

        # extract the normalizer
        if hasattr(policy_nn, "actor_obs_normalizer"):
            normalizer = policy_nn.actor_obs_normalizer
        elif hasattr(policy_nn, "student_obs_normalizer"):
            normalizer = policy_nn.student_obs_normalizer
        else:
            normalizer = None

        # export to JIT and ONNX
        export_policy_as_jit(policy_nn, normalizer=normalizer, path=export_model_dir, filename="policy.pt")
        export_policy_as_onnx(policy_nn, normalizer=normalizer, path=export_model_dir, filename="policy.onnx")

    dt = env.unwrapped.step_dt

    # reset environment
    obs = env.get_observations()
    timestep = 0
    wall_start = time.time()
    # simulate environment
    while simulation_app.is_running():
        start_time = time.time()
        # run everything in inference mode
        with torch.inference_mode():
            if keyboard is not None:
                keyboard.advance()
            obs = env.get_observations()
            # agent stepping
            actions = policy(obs)
            if args_cli.print_policy_data:
                scaled_actions = _scale_policy_action(actions, 0.25)
                policy_obs = _get_obs_group(obs, "policy")
                if args_cli.latest_frame:
                    latest_policy = _split_latest_frame(env, "policy", policy_obs)
                    obs_field = latest_policy if latest_policy is not None else policy_obs
                    label = "obs_policy_latest" if latest_policy is not None else "obs_policy"
                    print(
                        f"[POLICY] step={timestep} (latest frame only)\n"
                        f"{label}={_to_printable_policy_data(obs_field)}\n"
                        f"actions_raw={_to_printable_policy_data(actions)}\n"
                        f"actions_scaled={_to_printable_policy_data(scaled_actions)}",
                        flush=True,
                    )
                else:
                    critic_obs = _first_n_obs_elements(_get_obs_group(obs, "critic"), 45)
                    print(
                        f"[POLICY] step={timestep}\n"
                        f"obs_policy={_to_printable_policy_data(policy_obs)}\n"
                        f"obs_critic_first45={_to_printable_policy_data(critic_obs)}\n"
                        f"actions_raw={_to_printable_policy_data(actions)}\n"
                        f"actions_scaled={_to_printable_policy_data(scaled_actions)}",
                        flush=True,
                    )
            # env stepping
            obs, _, dones, _ = env.step(actions)
            if keyboard is not None:
                keyboard.advance()
            # reset recurrent states for episodes that have terminated
            if version.parse(installed_version) >= version.parse("4.0.0"):
                policy.reset(dones)
            else:
                policy_nn.reset(dones)
        timestep += 1
        if timestep % 100 == 0:
            elapsed = time.time() - wall_start
            print(f"[RTF] step={timestep} | SPS={timestep / elapsed:.1f} | RTF={timestep * dt / elapsed:.2f}x")
        if args_cli.video:
            # Exit the play loop after recording one video
            if timestep == args_cli.video_length:
                break

        # time delay for real-time evaluation
        sleep_time = dt - (time.time() - start_time)
        if args_cli.real_time and sleep_time > 0:
            time.sleep(sleep_time)

    # close the simulator
    env.close()


if __name__ == "__main__":
    # run the main function
    main()
    # close sim app
    simulation_app.close()
