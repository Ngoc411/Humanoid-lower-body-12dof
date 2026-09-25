# Copyright (c) 2022-2025, The Isaac Lab Project Developers (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

import gymnasium as gym

from . import agents

##
# Register Gym environments.
##


def _register_rsl_rl_env(task_id: str, env_cfg: str) -> None:
    gym.register(
        id=task_id,
        entry_point="isaaclab.envs:ManagerBasedRLEnv",
        disable_env_checker=True,
        kwargs={
            "env_cfg_entry_point": f"{__name__}.vin_rsl_rl_env_cfg:{env_cfg}",
            "rsl_rl_cfg_entry_point": f"{agents.__name__}.rsl_rl_ppo_cfg:PPORunnerCfg",
        },
    )


_register_rsl_rl_env("Template-Vin-Rsl-Rl-v0", "VinRslRlEnvCfg")
_register_rsl_rl_env("Template-Vin-Rsl-Rl-Flat-v0", "VinRslRlFlatEnvCfg")
_register_rsl_rl_env("Template-Vin-Rsl-Rl-Play-v0", "VinRslRlPlayEnvCfg")
_register_rsl_rl_env("Template-Vin-Rsl-Rl-Flat-Play-v0", "VinRslRlFlatPlayEnvCfg")
