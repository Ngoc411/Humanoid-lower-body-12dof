from __future__ import annotations

import sys
from pathlib import Path

RESULT = Path("pretrain_smoke_result.txt")


def log(message: str) -> None:
    with RESULT.open("a", encoding="utf-8") as f:
        f.write(message + "\n")
        f.flush()


RESULT.write_text("START\n", encoding="utf-8")

from isaaclab.app import AppLauncher

app_launcher = AppLauncher(headless=True)
simulation_app = app_launcher.app

try:
    repo_root = Path(__file__).resolve().parents[1]
    sys.path.insert(0, str(repo_root / "source" / "Vin_Rsl_Rl"))

    import torch
    from isaaclab.envs import ManagerBasedRLEnv

    from Vin_Rsl_Rl.tasks.manager_based.vin_rsl_rl.vin_rsl_rl_env_cfg import (
        VinRslRlEnvCfg,
        VinRslRlFlatEnvCfg,
    )

    for cfg_cls in (VinRslRlEnvCfg,):
        cfg = cfg_cls()
        cfg.scene.num_envs = 1
        cfg.observations.policy.enable_corruption = False
        cfg.events.push_robot = None
        cfg.curriculum.terrain_levels = None
        cfg.curriculum.command_vel = None

        log(f"CFG {cfg_cls.__name__}")
        env = ManagerBasedRLEnv(cfg)
        try:
            obs, extras = env.reset()
            log(f"OBS_KEYS {list(obs.keys())}")
            for group_name, group_obs in obs.items():
                log(f"OBS_SHAPE {group_name} {tuple(group_obs.shape)} finite={torch.isfinite(group_obs).all().item()}")

            action = torch.zeros(env.num_envs, env.action_manager.total_action_dim, device=env.device)
            for i in range(3):
                obs, reward, terminated, truncated, extras = env.step(action)
                finite_obs = all(torch.isfinite(v).all().item() for v in obs.values())
                finite_reward = torch.isfinite(reward).all().item()
                log(
                    "STEP "
                    f"{i} reward={reward.detach().cpu().tolist()} "
                    f"finite_obs={finite_obs} finite_reward={finite_reward} "
                    f"terminated={terminated.cpu().tolist()} truncated={truncated.cpu().tolist()}"
                )

            sensor = env.scene["feet_ground_contact"]
            log(f"CONTACT_BODY_NAMES {sensor.body_names}")
            log(f"CONTACT_AIR_TIME_SHAPE {tuple(sensor.data.current_air_time.shape)}")
            log(f"CONTACT_FORCE_SHAPE {tuple(sensor.data.net_forces_w.shape)}")
        finally:
            env.close()
except BaseException as exc:
    log(f"EXCEPTION {type(exc).__name__}: {exc!r}")
finally:
    log("CLOSING_APP")
    simulation_app.close()
