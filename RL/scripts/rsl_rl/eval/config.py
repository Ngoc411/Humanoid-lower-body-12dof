# Copyright (c) 2025-2026, The RoboLab Project Developers.
# SPDX-License-Identifier: BSD-3-Clause
"""Static configuration for the 3-phase AMP evaluation.

Everything that is robot-/task-specific (body names, thresholds, sweep grids)
lives here so the metric and harness code stays generic. Names were taken from:
  - robolab/assets/robots/chaos.py            (joint/actuator definitions)
  - robolab/tasks/manager_based/amp/...       (foot = .*_ankle_roll, base_link)
  - data/motions/chaos_walk_*.pkl             (reference clips + measured speeds)
"""

from __future__ import annotations

import math

# --------------------------------------------------------------------------- #
# Robot wiring (verified against chaos.py / atom01_amp_env_cfg.py)
# --------------------------------------------------------------------------- #
# 12-DOF legs-only order as registered in the articulation. We still read the
# live joint_names at runtime and remap, but this is the expected layout.
JOINT_ORDER = [
    "left_hip_pitch_joint", "right_hip_pitch_joint",
    "left_hip_roll_joint", "right_hip_roll_joint",
    "left_hip_yaw_joint", "right_hip_yaw_joint",
    "left_knee_joint", "right_knee_joint",
    "left_ankle_pitch_joint", "right_ankle_pitch_joint",
    "left_ankle_roll_joint", "right_ankle_roll_joint",
]

# Left/right pairs used for gait symmetry / phase-error metrics.
LEFT_JOINTS = [j for j in JOINT_ORDER if j.startswith("left_")]
RIGHT_JOINTS = [j.replace("left_", "right_") for j in LEFT_JOINTS]

# Body-name regexes (Isaac Lab SceneEntityCfg style) for find_bodies().
FOOT_BODY_REGEX = ".*_ankle_roll.*"      # left_ankle_roll_link / right_ankle_roll_link
LEFT_FOOT_REGEX = "left_ankle_roll.*"
RIGHT_FOOT_REGEX = "right_ankle_roll.*"
BASE_BODY_NAME = "base_link"

CONTACT_SENSOR = "contact_forces"
ROBOT_ASSET = "robot"
ANIMATION_TERM = "animation"             # AnimationManager term name

# --------------------------------------------------------------------------- #
# Physical constants / thresholds
# --------------------------------------------------------------------------- #
GRAVITY = 9.81
CONTACT_FORCE_THRESHOLD = 1.0            # N, foot considered in contact above this
FOOT_SLIDING_THRESHOLD = 0.01            # m/s, AMP sliding pass/fail
JOINT_CORR_GOOD = 0.9                    # Pearson r target

# Fall definition.
# IMPORTANT: per chaos.py the base_link origin is at FOOT-plate level (SolidWorks
# URDF export convention), so base_z ~ 0 when standing upright. Use tilt as the
# primary fall signal; base_z only catches catastrophic falls (robot through floor).
FALL_BASE_HEIGHT = -0.5                  # m, anything below this = catastrophic
FALL_TILT_DEG = 45.0                     # torso tilt above this -> fallen
FALL_TILT_RAD = math.radians(FALL_TILT_DEG)

# Steady-state window: drop the first SETTLE_S seconds of each fixed-command
# segment before computing tracking/energy metrics (removes startup transient).
SETTLE_S = 1.0

# Number of real consecutive gait cycles to store for the motion-quality plot.
MOTION_N_CYCLES = 5
MOTION_N_PHASE_PTS = 100          # resample resolution per cycle

# --------------------------------------------------------------------------- #
# Reference clips (steady-state forward speed -- the LAST 1s of each clip,
# excluding the ramp-up portion. The naive mean-over-clip underestimates the
# walking speed because each clip starts from a stand-still.)
# --------------------------------------------------------------------------- #
REFERENCE_CLIPS = [
    {"clip": "chaos_walk_50",      "cmd_vx": 0.43},  # steady ~0.43, mean-all 0.29
    {"clip": "chaos_walk_forward", "cmd_vx": 0.63},  # steady ~0.63, mean-all 0.43
    {"clip": "chaos_walk_100",     "cmd_vx": 1.06},  # steady ~1.06, mean-all 0.72
]
# Seconds at the start of each clip to ignore when phase-averaging the reference
# trajectory (ramp-up from stationary).
REF_RAMP_S = 0.5

# --------------------------------------------------------------------------- #
# Phase 3 (task) velocity sweep  -- dense per-axis, full training range + extrap.
# Training ranges: vx (-0.8, 2.0), vy (-0.8, 0.8), wz (-0.8, 0.8)
# --------------------------------------------------------------------------- #
SWEEP_VX = [-0.8, -0.4, 0.0, 0.3, 0.43, 0.63, 1.06, 1.5, 2.0]
SWEEP_VY = [-0.8, -0.4, 0.0, 0.4, 0.8]
SWEEP_WZ = [-0.8, -0.4, 0.0, 0.4, 0.8]
DATASET_VX = {0.43, 0.63, 1.06}          # steady-state speeds of the 3 forward clips

# (vx, vy, wz) tuples, deduplicated, keeping the all-zero point once.
def build_sweep_conditions() -> list[dict]:
    conds: list[dict] = []
    seen: set = set()

    def add(vx, vy, wz, axis):
        key = (round(vx, 3), round(vy, 3), round(wz, 3))
        if key in seen:
            return
        seen.add(key)
        conds.append({
            "condition_id": f"{axis}_vx{vx:+.2f}_vy{vy:+.2f}_wz{wz:+.2f}",
            "cmd": (vx, vy, wz), "axis": axis,
            "in_dataset_range": abs(vx) in DATASET_VX and vy == 0.0 and wz == 0.0,
        })

    for vx in SWEEP_VX:
        add(vx, 0.0, 0.0, "vx")
    for vy in SWEEP_VY:
        add(0.0, vy, 0.0, "vy")
    for wz in SWEEP_WZ:
        add(0.0, 0.0, wz, "wz")
    return conds

# Replicas per condition: how many parallel envs share one condition (mean/std).
# Required num_envs >= SWEEP_ENVS_PER_CONDITION * n_conditions (~17) = ~1700
SWEEP_ENVS_PER_CONDITION = 100

# Velocity ramp-up for high-speed sweep conditions.
# Envs commanded |vx| > this threshold will ramp linearly from 0 → target
# over SWEEP_RAMP_S seconds before holding at target speed.
SWEEP_RAMP_THRESHOLD_VX = 1.0            # m/s — conditions above this get a ramp
SWEEP_RAMP_START_VX = 0.5               # m/s — ramp begins from this speed (not 0)
SWEEP_RAMP_S = 2.0                       # seconds to ramp (must be >= SETTLE_S)

# Step-response probes: hold base speed, then step the command mid-episode.
STEP_RESPONSE_PROBES = [
    {"axis": "vx", "from": 0.0, "to": 1.0},
    {"axis": "vx", "from": 0.5, "to": 1.5},
    {"axis": "vy", "from": 0.0, "to": 0.5},
    {"axis": "wz", "from": 0.0, "to": 0.5},
]
STEP_HOLD_S = 3.0                        # seconds to stabilise before the step
STEP_WINDOW_S = 3.0                      # seconds of transient to record after

# --------------------------------------------------------------------------- #
# Phase 4 (robustness)
# --------------------------------------------------------------------------- #
ROBUST_CMD_VX = [0.0, 0.5, 1.0]          # user-requested command marks
PUSH_VELS = [0.5, 1.0, 1.5, 2.0, 2.5]    # m/s root-velocity impulse magnitudes
PUSH_DIRECTIONS = {                       # name -> (vx, vy) unit direction
    "forward": (1.0, 0.0),
    "backward": (-1.0, 0.0),
    "left": (0.0, 1.0),
    "right": (0.0, -1.0),
}
PUSH_TRIALS_PER_CELL = 100                # envs per (cmd_vx, push_vel, direction)
# Required num_envs >= PUSH_TRIALS_PER_CELL * n_cells (3×5×4=60) = 6000
PUSH_SETTLE_S = 3.0                       # walk normally before the push
PUSH_RECOVERY_WINDOW_S = 3.0             # window to judge survival / recovery
PUSH_RECOVERY_TOL = 0.10                  # m, CoM error below this = recovered
SURVIVAL_THRESHOLD = 0.5                  # mean survival level for max-recoverable

NOISE_SIGMAS = [0.0, 0.01, 0.05, 0.10, 0.20]
NOISE_SURVIVAL_TOL = 0.9                  # threshold sigma = where survival < this

# Terrain generalization is optional (plane only in current scene cfg).
ENABLE_TERRAIN = False
