# Copyright (c) 2025-2026, The RoboLab Project Developers.
# SPDX-License-Identifier: BSD-3-Clause
"""The three evaluation phases.

Each phase: (1) assigns a fixed command to every env (each env = one condition),
(2) rolls the policy out collecting per-step signals, (3) computes metrics on ALL
envs and aggregates per condition, (4) returns a summary dict + a small set of
representative-env time-series for parquet.

Phase 2 (motion)  : 3 runs, command = clip speed, phase-normalized joint tracking
Phase 3 (task)    : dense per-axis velocity sweep + step-response + energy/survival
Phase 4 (robust)  : push (set-root-velocity) x cmd_vx x direction + obs-noise sweep
"""

from __future__ import annotations

import os
import numpy as np
import torch

from . import config as C
from . import metrics as M
from .accumulators import EpisodeRecorder, RobotSampler


# --------------------------------------------------------------------------- #
# Rollout + command helpers
# --------------------------------------------------------------------------- #
def _set_commands(sampler: RobotSampler, cmd: torch.Tensor) -> None:
    """Force a per-env (vx, vy, wz) command, overriding the command manager."""
    term = sampler.env.command_manager.get_term("base_velocity")
    term.vel_command_b[:, :3] = cmd


def _reset_env(env):
    """Reset inside inference_mode (sim tensors are inference tensors)."""
    with torch.inference_mode():
        env.reset()


def _rollout(env, policy, policy_nn, sampler, recorder, n_steps,
             cmd=None, cmd_fn=None, pre_action=None, post_step=None):
    """Generic rollout.

    `cmd` (N,3): fixed command re-asserted every step.
    `cmd_fn(step) -> Tensor`: time-varying command; takes priority over `cmd`.
    The whole loop runs inside inference_mode so in-place sim writes are permitted.
    """
    with torch.inference_mode():
        obs = env.get_observations()
        if isinstance(obs, tuple):
            obs = obs[0]
        current_cmd = cmd_fn(0) if cmd_fn is not None else cmd
        if current_cmd is not None:
            _set_commands(sampler, current_cmd)
        for step in range(n_steps):
            if pre_action is not None:
                obs = pre_action(step, obs)
            actions = policy(obs)
            obs, _, dones, _ = env.step(actions)
            if isinstance(obs, tuple):
                obs = obs[0]
            policy_nn.reset(dones)
            current_cmd = cmd_fn(step) if cmd_fn is not None else cmd
            if current_cmd is not None:
                _set_commands(sampler, current_cmd)
            rec = sampler.sample()
            rec["dones"] = dones.float()
            recorder.append(**rec)
            if post_step is not None:
                post_step(step)
    return recorder


def _assign_round_robin(conditions, num_envs):
    """env_id -> condition index, cycling so every condition gets ~equal envs."""
    n = len(conditions)
    idx = np.array([k % n for k in range(num_envs)], dtype=int)
    return idx


def _steady_slice(arr, fps, settle_s=C.SETTLE_S):
    """Drop the first settle_s seconds (axis 0 = time within one env array)."""
    start = int(settle_s * fps)
    return arr[start:] if arr.shape[0] > start + 2 else arr


# --------------------------------------------------------------------------- #
# Per-env locomotion metric bundle (used by phase 2 & 3)
# --------------------------------------------------------------------------- #
def _locomotion_metrics(env_arr, fps, mass, cmd):
    """Compute metrics for ONE env. env_arr: dict key -> (T, ...) for this env."""
    jp = env_arr["joint_pos"]                    # (T, J)
    jv = env_arr["joint_vel"]
    tau = env_arr["torque"]
    rpos = env_arr["root_pos"]                    # (T, 3)
    vb = env_arr["root_lin_vel_b"]               # (T, 3)
    foot_pos = env_arr["foot_pos"]               # (T, 2, 3)
    foot_vel = env_arr["foot_vel"]               # (T, 2, 3)
    contact = env_arr["foot_contact"] > C.CONTACT_FORCE_THRESHOLD   # (T, 2)
    base_z = env_arr["base_z"]
    tilt = env_arr["tilt"]

    lc, rc = contact[:, 0], contact[:, 1]
    ls = M.heel_strikes(lc)

    # velocity tracking (commanded axes)
    err = np.stack([vb[:, 0] - cmd[0], vb[:, 1] - cmd[1]], axis=1)
    vel_rmse = float(np.sqrt(np.mean(np.sum(err ** 2, axis=1))))
    vel_mae = float(np.mean(np.abs(err)))

    # energy / CoT over the window (skip CoT for standing / pure rotation tasks)
    dist = float(np.linalg.norm(rpos[-1, :2] - rpos[0, :2]))
    energy = M.total_joint_energy(tau, jv, 1.0 / fps)
    speed = float(np.linalg.norm(vb[:, :2], axis=1).mean())
    cot = M.cost_of_transport(energy, mass, dist) if dist > 0.1 else float("nan")
    power = float(np.mean(np.abs(tau * jv)))

    # gait
    foot_sliding = _safe_max(
        M.foot_sliding(foot_vel[:, 0, :2], lc),
        M.foot_sliding(foot_vel[:, 1, :2], rc),
    )
    l_stride = M.stride_length(foot_pos[:, 0, :2], ls)
    r_stride = M.stride_length(foot_pos[:, 1, :2], M.heel_strikes(rc))

    return {
        "vel_rmse": vel_rmse, "vel_mae": vel_mae,
        "cot": cot, "power_w": power, "energy_j": energy, "distance_m": dist,
        "survival": float(M.survived(base_z, tilt, C.FALL_BASE_HEIGHT, C.FALL_TILT_RAD)),
        "foot_sliding_ms": foot_sliding,
        "step_freq_hz": M.step_frequency(lc, rc, fps),
        "duty_factor": M.duty_factor(lc),
        "double_support_ratio": M.double_support_ratio(lc, rc),
        "stride_length_m": _safe_mean(l_stride, r_stride),
        "symmetry_index": M.symmetry_index(l_stride, r_stride),
        "foot_clearance_m": _safe_max(
            M.foot_clearance(foot_pos[:, 0, 2], lc),
            M.foot_clearance(foot_pos[:, 1, 2], rc),
        ),
        **M.com_stability(rpos, env_arr["root_lin_vel_w"], fps),
    }


def _safe_mean(*vals):
    arr = [v for v in vals if v is not None and not (isinstance(v, float) and np.isnan(v))]
    return float(np.mean(arr)) if arr else float("nan")


def _safe_max(*vals):
    arr = [v for v in vals if v is not None and not (isinstance(v, float) and np.isnan(v))]
    return float(max(arr)) if arr else float("nan")


def _env_view(stacked, e):
    """Slice the stacked (N, T, ...) dict down to one env -> (T, ...)."""
    return {k: v[e] for k, v in stacked.items()}


# --------------------------------------------------------------------------- #
# PHASE 2 — Motion quality
# --------------------------------------------------------------------------- #
def _load_reference_cycle(motions_dir, clip, sim_joint_names, n_phase=100):
    """Phase-average a reference clip's joint trajectory (from its own pkl).

    Returns (cycle (P, J) reordered to sim_joint_names, names) or (None, None).
    """
    import joblib
    path = os.path.join(motions_dir, clip + ".pkl")
    if not os.path.exists(path):
        print(f"[eval] reference clip not found: {path}")
        return None, None
    d = joblib.load(path)
    dof = np.asarray(d["dof_pos"], float)                      # (T, 12)
    dof_names = list(d.get("dof_names", C.JOINT_ORDER))
    kbp = np.asarray(d["key_body_pos"], float)                # (T, K, 3): foot z for contact
    fps = float(d["fps"])
    # Drop the initial ramp-up where the robot is still stationary; the AMP clips
    # start from a stand-still and only reach steady walking after ~0.5s.
    drop = int(C.REF_RAMP_S * fps)
    dof = dof[drop:]
    kbp = kbp[drop:]
    # derive contact from left-foot height (key body 0 = left_ankle_roll_link)
    lz = kbp[:, 0, 2]
    contact = lz < (lz.min() + 0.015)
    strikes = M.heel_strikes(contact, min_gap=int(0.1 * fps))
    cycle, _ = M.phase_average(dof, strikes, n_phase)         # (P, 12) in dof_names order
    # Reorder columns to sim joint order. The pkl strips the "_joint" suffix
    # (e.g. "left_hip_pitch"), while the articulation keeps it
    # (e.g. "left_hip_pitch_joint"), so normalize both sides before matching.
    _norm = lambda n: n.replace("_joint", "")
    name_to_col = {_norm(n): i for i, n in enumerate(dof_names)}
    cols = [name_to_col[_norm(n)] for n in sim_joint_names if _norm(n) in name_to_col]
    used_names = [n for n in sim_joint_names if _norm(n) in name_to_col]
    if not cols:
        print(f"[eval] WARN: joint names don't overlap for {clip}: "
              f"pkl={dof_names[:3]}... sim={sim_joint_names[:3]}...")
        return None, None
    return cycle[:, cols], used_names


def run_motion_phase(env, policy, policy_nn, fps, mass, motions_dir, n_steps):
    sampler = RobotSampler(env, C)
    num_envs = sampler.env.num_envs
    runs = []
    ts_series, ts_envs, ts_conds = {}, [], []

    for spec in C.REFERENCE_CLIPS:
        clip, vx = spec["clip"], spec["cmd_vx"]
        cmd = torch.zeros((num_envs, 3), device=sampler.device)
        cmd[:, 0] = vx
        _reset_env(env)
        rec = EpisodeRecorder()
        _rollout(env, policy, policy_nn, sampler, rec, n_steps, cmd=cmd)
        stacked = rec.stack()

        # reference cycle from pkl
        ref_cycle, names = _load_reference_cycle(motions_dir, clip, sampler.joint_names)

        # per-env phase-normalized joint tracking
        per_joint_rmse, per_joint_corr, learned_cycles = [], [], []
        loco = []
        for e in range(num_envs):
            ev = _env_view(stacked, e)
            if not bool(M.survived(ev["base_z"], ev["tilt"], C.FALL_BASE_HEIGHT, C.FALL_TILT_RAD)):
                continue
            lc = ev["foot_contact"][:, 0] > C.CONTACT_FORCE_THRESHOLD
            ls = M.heel_strikes(lc)
            lcyc, n = M.phase_average(ev["joint_pos"], ls)
            if n >= 1 and ref_cycle is not None:
                learned_cycles.append(lcyc)
                per_joint_rmse.append(M.rmse(lcyc, ref_cycle, axis=0))
                per_joint_corr.append([M.pearson(lcyc[:, j], ref_cycle[:, j])
                                       for j in range(ref_cycle.shape[1])])
            loco.append(_locomotion_metrics(ev, fps, mass, (vx, 0.0, 0.0)))

        joint_tracking = {}
        if per_joint_rmse:
            rmse_arr = np.nanmean(np.stack(per_joint_rmse), axis=0)
            corr_arr = np.nanmean(np.stack(per_joint_corr), axis=0)
            for j, nm in enumerate(names):
                joint_tracking[nm] = {"angle_rmse": float(rmse_arr[j]),
                                      "angle_corr": float(corr_arr[j])}
            joint_tracking["_aggregate"] = {
                "mean_angle_rmse": float(np.nanmean(rmse_arr)),
                "mean_corr": float(np.nanmean(corr_arr)),
            }

        run = {
            "clip": clip, "cmd_vx": vx, "n_envs_survived": len(loco),
            "joint_tracking": joint_tracking,
            "gait_quality": _agg_dict(loco, ["stride_length_m", "step_freq_hz", "duty_factor",
                                             "double_support_ratio", "symmetry_index",
                                             "foot_clearance_m", "foot_sliding_ms"]),
            "com_stability": _agg_dict(loco, ["height_mean", "height_std",
                                              "lateral_deviation_max", "velocity_smoothness"]),
            "phase_error": _phase_error_avg(stacked, fps),
        }
        if ref_cycle is not None:
            # Extract real consecutive cycles from one representative surviving env
            # (not averaged across envs) so the plot shows actual cycle-to-cycle variation.
            rep = _first_survivor(stacked)
            rep_cycles_arr = None
            n_real = 0
            if rep is not None:
                ev_rep = _env_view(stacked, rep)
                lc_rep = ev_rep["foot_contact"][:, 0] > C.CONTACT_FORCE_THRESHOLD
                strikes_rep = M.heel_strikes(lc_rep)
                rep_cycles_arr, n_real = M.extract_consecutive_cycles(
                    ev_rep["joint_pos"], strikes_rep,
                    n_cycles=C.MOTION_N_CYCLES, n_phase=C.MOTION_N_PHASE_PTS,
                )
            if rep_cycles_arr is not None:
                # Reference: tile 1 cycle to match n_real cycles for plotting
                ref_tiled = np.tile(ref_cycle, (n_real, 1))
                run["cycles"] = {
                    "learned": rep_cycles_arr.tolist(),
                    "reference": ref_tiled.tolist(),
                    "joint_names": names,
                    "n_cycles": n_real,
                    "n_phase_pts": C.MOTION_N_PHASE_PTS,
                }
        runs.append(run)

        # one representative survivor env for parquet
        rep = _first_survivor(stacked)
        if rep is not None:
            _collect_ts(ts_series, ts_envs, ts_conds, stacked, rep, clip, sampler.joint_names)

    summary = {"meta": {"control_hz": fps, "episode_steps": n_steps,
                        "n_envs": num_envs, "n_runs": len(runs)}, "runs": runs}
    return summary, (ts_series, ts_envs, ts_conds)


def _phase_error_avg(stacked, fps):
    vals = []
    for e in range(stacked["foot_contact"].shape[0]):
        c = stacked["foot_contact"][e] > C.CONTACT_FORCE_THRESHOLD
        pe = M.phase_error_lr(c[:, 0], c[:, 1], fps)
        if not np.isnan(pe["lag_fraction"]):
            vals.append(pe["lag_fraction"])
    if not vals:
        return {"lag_fraction": float("nan"), "ideal": 0.5}
    return {"lag_fraction": float(np.mean(vals)), "ideal": 0.5}


# --------------------------------------------------------------------------- #
# PHASE 3 — Task metrics (velocity sweep + step response)
# --------------------------------------------------------------------------- #
def run_task_phase(env, policy, policy_nn, fps, mass, n_steps):
    sampler = RobotSampler(env, C)
    num_envs = sampler.env.num_envs
    conditions = C.build_sweep_conditions()
    cond_idx = _assign_round_robin(conditions, num_envs)

    cmd = torch.zeros((num_envs, 3), device=sampler.device)
    for e in range(num_envs):
        cmd[e] = torch.tensor(conditions[cond_idx[e]]["cmd"], device=sampler.device)

    # Ramp mask: envs where |vx| > threshold get a linear ramp-up
    ramp_steps = int(C.SWEEP_RAMP_S * fps)
    needs_ramp = torch.zeros(num_envs, dtype=torch.bool, device=sampler.device)
    for e in range(num_envs):
        if abs(conditions[cond_idx[e]]["cmd"][0]) > C.SWEEP_RAMP_THRESHOLD_VX:
            needs_ramp[e] = True

    ramp_start = C.SWEEP_RAMP_START_VX

    def cmd_fn(step: int) -> torch.Tensor:
        if step >= ramp_steps or not needs_ramp.any():
            return cmd
        alpha = float(step) / ramp_steps   # 0 → 1 linearly over ramp_steps
        ramped = cmd.clone()
        # Ramp from RAMP_START_VX → target (not from 0)
        # vx(t) = start + alpha * (target - start)
        start_cmd = torch.full_like(cmd, 0.0)
        start_cmd[needs_ramp, 0] = ramp_start
        ramped[needs_ramp] = start_cmd[needs_ramp] + alpha * (cmd[needs_ramp] - start_cmd[needs_ramp])
        return ramped

    # Metrics settle window: must be >= ramp duration so ramped envs are stable
    settle_s = max(C.SETTLE_S, C.SWEEP_RAMP_S)

    _reset_env(env)
    rec = EpisodeRecorder()
    _rollout(env, policy, policy_nn, sampler, rec, n_steps, cmd_fn=cmd_fn)
    stacked = rec.stack()

    # steady-state metrics per env, grouped by condition
    per_cond: dict[int, list] = {}
    for e in range(num_envs):
        ev = {k: _steady_slice(v[e], fps, settle_s) for k, v in stacked.items()}
        cdef = conditions[cond_idx[e]]
        per_cond.setdefault(cond_idx[e], []).append(
            _locomotion_metrics(ev, fps, mass, cdef["cmd"]))

    table = []
    for ci, rows in sorted(per_cond.items()):
        cdef = conditions[ci]
        vx, vy, wz = cdef["cmd"]
        survivors = [r for r in rows if r["survival"] > 0.5] or rows
        perf = {k: float(np.nanmean([r[k] for r in survivors]))
                for k in ("vel_rmse", "vel_mae", "cot", "power_w",
                          "foot_sliding_ms", "step_freq_hz", "duty_factor")}
        perf["survival"] = float(np.nanmean([r["survival"] for r in rows]))
        table.append({"condition_id": cdef["condition_id"], "axis": cdef["axis"],
                      "cmd_vx": vx, "cmd_vy": vy, "cmd_wz": wz,
                      "in_dataset_range": cdef["in_dataset_range"],
                      "n_envs": len(rows), "n_survived": len(survivors),
                      **{k: round(v, 5) for k, v in perf.items()}})

    # survival summary over all envs/conditions
    surv = [r["survival"] for rows in per_cond.values() for r in rows]
    survival = {"survival_rate_pct": 100.0 * float(np.mean(surv)),
                "fall_rate_pct": 100.0 * (1 - float(np.mean(surv)))}

    step_resp, sr_series = _run_step_response(env, policy, policy_nn, sampler, fps)

    summary = {
        "meta": {"control_hz": fps, "robot_mass_kg": mass, "n_envs": num_envs,
                 "episode_steps": n_steps, "settle_s": C.SETTLE_S},
        "velocity_sweep": {"axes": ["vx", "vy", "wz"], "table": table},
        "step_response": step_resp,
        "survival": survival,
    }
    # representative envs: one per axis
    ts_series, ts_envs, ts_conds = {}, [], []
    for axis in ("vx", "vy", "wz"):
        for e in range(num_envs):
            if conditions[cond_idx[e]]["axis"] == axis:
                _collect_ts(ts_series, ts_envs, ts_conds, stacked, e,
                            conditions[cond_idx[e]]["condition_id"], sampler.joint_names)
                break
    return summary, (ts_series, ts_envs, ts_conds), sr_series


def _run_step_response(env, policy, policy_nn, sampler, fps):
    """Assign each probe to a block of envs; hold, then step the command."""
    num_envs = sampler.env.num_envs
    probes = C.STEP_RESPONSE_PROBES
    probe_idx = _assign_round_robin(probes, num_envs)
    axis_col = {"vx": 0, "vy": 1, "wz": 2}

    cmd0 = torch.zeros((num_envs, 3), device=sampler.device)
    for e in range(num_envs):
        p = probes[probe_idx[e]]
        cmd0[e, axis_col[p["axis"]]] = p["from"]

    _reset_env(env)
    hold = int(C.STEP_HOLD_S * fps)
    win = int(C.STEP_WINDOW_S * fps)
    rec = EpisodeRecorder()

    # phase A: hold the 'from' command
    _rollout(env, policy, policy_nn, sampler, EpisodeRecorder(), hold, cmd=cmd0)
    # phase B: switch to 'to' and record the transient
    cmd1 = cmd0.clone()
    for e in range(num_envs):
        p = probes[probe_idx[e]]
        cmd1[e, axis_col[p["axis"]]] = p["to"]
    _rollout(env, policy, policy_nn, sampler, rec, win, cmd=cmd1)
    stacked = rec.stack()

    results, series, s_envs, s_conds = [], {}, [], []
    for pi, p in enumerate(probes):
        col = axis_col[p["axis"]]
        vel_key = "root_lin_vel_b" if p["axis"] in ("vx", "vy") else "root_ang_vel_b"
        envs = [e for e in range(num_envs) if probe_idx[e] == pi]
        if not envs:
            continue
        sigs = []
        for e in envs:
            sig = stacked.get(vel_key, stacked["root_lin_vel_b"])[e][:, col if p["axis"] != "wz" else 2]
            sigs.append(sig)
        mean_sig = np.nanmean(np.stack(sigs), axis=0)
        sr = M.step_response_metrics(mean_sig, fps, p["from"], p["to"])
        results.append({"axis": p["axis"], "from": p["from"], "to": p["to"], **sr})
        # representative series
        series.setdefault("v_signal", []).append(mean_sig)
        s_envs.append(envs[0])
        s_conds.append(f"step_{p['axis']}_{p['from']}_{p['to']}")
    if series:
        series = {"v_signal": np.stack(series["v_signal"])}
    return results, (series, s_envs, s_conds)


# --------------------------------------------------------------------------- #
# PHASE 4 — Robustness (push + noise)
# --------------------------------------------------------------------------- #
def run_robust_phase(env, policy, policy_nn, fps, mass):
    sampler = RobotSampler(env, C)
    push = _run_push(env, policy, policy_nn, sampler, fps)
    noise = _run_noise(env, policy, policy_nn, sampler, fps)
    summary = {
        "meta": {"control_hz": fps, "push_mode": "set_root_velocity",
                 "trials_per_cell": C.PUSH_TRIALS_PER_CELL,
                 "cmd_vx_levels": C.ROBUST_CMD_VX},
        "push_robustness": push[0],
        "noise_robustness": noise,
    }
    if C.ENABLE_TERRAIN:
        summary["terrain_generalization"] = {"table": []}
    return summary, push[1]


def _run_push(env, policy, policy_nn, sampler, fps):
    num_envs = sampler.env.num_envs
    # build cells: (cmd_vx, push_vel, direction)
    cells = [(vx, pv, d) for vx in C.ROBUST_CMD_VX
             for pv in C.PUSH_VELS for d in C.PUSH_DIRECTIONS]
    cell_idx = _assign_round_robin(cells, num_envs)

    cmd = torch.zeros((num_envs, 3), device=sampler.device)
    for e in range(num_envs):
        cmd[e, 0] = cells[cell_idx[e]][0]

    _reset_env(env)
    settle = int(C.PUSH_SETTLE_S * fps)
    win = int(C.PUSH_RECOVERY_WINDOW_S * fps)
    robot = sampler.robot

    # settle walking
    _rollout(env, policy, policy_nn, sampler, EpisodeRecorder(), settle, cmd=cmd)

    # CoM reference right before push
    com_ref = sampler.root_pos_local().clone()

    def apply_push(step):
        if step != 0:
            return
        root_vel = robot.data.root_vel_w.clone()        # (N, 6)
        for e in range(num_envs):
            _, pv, d = cells[cell_idx[e]]
            dx, dy = C.PUSH_DIRECTIONS[d]
            root_vel[e, 0] += pv * dx
            root_vel[e, 1] += pv * dy
        robot.write_root_velocity_to_sim(root_vel)

    rec = EpisodeRecorder()
    _rollout(env, policy, policy_nn, sampler, rec, win, cmd=cmd,
             post_step=lambda s: apply_push(s))
    stacked = rec.stack()

    # survival + recovery per env
    com_ref_np = com_ref.cpu().numpy()
    per_cell: dict[int, dict] = {}
    for e in range(num_envs):
        ci = int(cell_idx[e])
        surv = M.survived(stacked["base_z"][e], stacked["tilt"][e],
                          C.FALL_BASE_HEIGHT, C.FALL_TILT_RAD)
        com_err = np.linalg.norm(stacked["root_pos"][e][:, :2] - com_ref_np[e, :2], axis=1)
        rt = M.recovery_time(com_err, fps, C.PUSH_RECOVERY_TOL)
        d = per_cell.setdefault(ci, {"surv": [], "rt": [], "disp": [], "tilt": []})
        d["surv"].append(float(surv))
        d["rt"].append(rt)
        d["disp"].append(float(com_err.max()))
        d["tilt"].append(float(stacked["tilt"][e].max()))

    # assemble table: rows keyed by (cmd_vx, push_vel), columns = directions
    dirs = list(C.PUSH_DIRECTIONS)
    grouped: dict[tuple, dict] = {}
    for ci, vals in per_cell.items():
        vx, pv, d = cells[ci]
        key = (vx, pv)
        grouped.setdefault(key, {})[d] = float(np.mean(vals["surv"]))
    table = []
    for (vx, pv), dvals in sorted(grouped.items()):
        row = {"cmd_vx": vx, "push_vel_ms": pv}
        for d in dirs:
            row[d] = round(dvals.get(d, float("nan")), 4)
        row["mean"] = round(float(np.nanmean(list(dvals.values()))), 4)
        table.append(row)

    # max recoverable velocity per cmd_vx (interpolate survival == threshold)
    max_rec = {}
    for vx in C.ROBUST_CMD_VX:
        pts = sorted([(r["push_vel_ms"], r["mean"]) for r in table if r["cmd_vx"] == vx])
        max_rec[f"cmd_vx_{vx}"] = _interp_threshold(pts, C.SURVIVAL_THRESHOLD)

    recovery = {
        "mean_recovery_time_s": float(np.nanmean(
            [rt for d in per_cell.values() for rt in d["rt"]])),
        "max_com_displacement_m": float(np.nanmax(
            [x for d in per_cell.values() for x in d["disp"]])),
        "max_torso_tilt_deg": float(np.degrees(np.nanmax(
            [x for d in per_cell.values() for x in d["tilt"]]))),
    }
    push_summary = {"directions": dirs, "table": table,
                    "max_recoverable_vel_ms": max_rec, "recovery": recovery}

    # representative time-series: one survivor + one faller
    ts_series, ts_envs, ts_conds = {}, [], []
    for e in range(num_envs):
        com_err = np.linalg.norm(stacked["root_pos"][e][:, :2] - com_ref_np[e, :2], axis=1)
        series_e = {"com_err": com_err, "tilt": stacked["tilt"][e], "base_z": stacked["base_z"][e]}
        _append_series(ts_series, series_e)
        ts_envs.append(e)
        vx, pv, d = cells[cell_idx[e]]
        ts_conds.append(f"push_vx{vx}_pv{pv}_{d}")
        if len(ts_envs) >= 8:                       # cap representative count
            break
    if ts_series:
        ts_series = {k: np.stack(v) for k, v in ts_series.items()}
    return push_summary, (ts_series, ts_envs, ts_conds)


def _run_noise(env, policy, policy_nn, sampler, fps):
    """One rollout per sigma (scalar noise, no per-env broadcasting).

    Each rollout: every env runs the SAME sigma; cmd_vx is varied across envs
    so survival/vel_rmse is averaged over the three cmd marks at that sigma.
    """
    num_envs = sampler.env.num_envs
    cmd_vx_idx = [k % len(C.ROBUST_CMD_VX) for k in range(num_envs)]
    cmd = torch.zeros((num_envs, 3), device=sampler.device)
    for e in range(num_envs):
        cmd[e, 0] = C.ROBUST_CMD_VX[cmd_vx_idx[e]]
    n_steps = int((C.PUSH_SETTLE_S + C.PUSH_RECOVERY_WINDOW_S) * fps)

    table = []
    for sig in C.NOISE_SIGMAS:
        sig_scalar = float(sig)

        def add_noise(step, obs, _s=sig_scalar):
            if _s == 0.0:
                return obs
            return obs + _s * torch.randn_like(obs)

        _reset_env(env)
        rec = EpisodeRecorder()
        _rollout(env, policy, policy_nn, sampler, rec, n_steps,
                 cmd=cmd, pre_action=add_noise)
        stacked = rec.stack()

        surv, rmse_vals = [], []
        for e in range(num_envs):
            ev = {k: _steady_slice(v[e], fps) for k, v in stacked.items()}
            vx = C.ROBUST_CMD_VX[cmd_vx_idx[e]]
            err = np.stack([ev["root_lin_vel_b"][:, 0] - vx,
                            ev["root_lin_vel_b"][:, 1]], axis=1)
            surv.append(float(M.survived(ev["base_z"], ev["tilt"],
                                         C.FALL_BASE_HEIGHT, C.FALL_TILT_RAD)))
            rmse_vals.append(float(np.sqrt(np.mean(np.sum(err ** 2, axis=1)))))
        table.append({"sigma": sig,
                      "survival": round(float(np.mean(surv)), 4),
                      "vel_rmse": round(float(np.nanmean(rmse_vals)), 4)})

    thr = next((r["sigma"] for r in table if r["survival"] < C.NOISE_SURVIVAL_TOL), None)
    return {"tolerance_threshold_sigma": thr, "table": table}


# --------------------------------------------------------------------------- #
# small shared helpers
# --------------------------------------------------------------------------- #
def _interp_threshold(points, level):
    """First x where the (sorted) curve crosses below `level`, linearly interp."""
    for i in range(1, len(points)):
        x0, y0 = points[i - 1]
        x1, y1 = points[i]
        if y0 >= level >= y1 and y0 != y1:
            return round(x0 + (level - y0) * (x1 - x0) / (y1 - y0), 4)
    return points[-1][0] if points and points[-1][1] >= level else None


def _agg_dict(rows, keys):
    out = {}
    for k in keys:
        vals = np.asarray([r[k] for r in rows], float)
        vals = vals[~np.isnan(vals)]
        out[k] = {"mean": float(vals.mean()), "std": float(vals.std())} if vals.size else None
    return out


def _first_survivor(stacked):
    for e in range(stacked["base_z"].shape[0]):
        if M.survived(stacked["base_z"][e], stacked["tilt"][e],
                      C.FALL_BASE_HEIGHT, C.FALL_TILT_RAD):
            return e
    return 0 if stacked["base_z"].shape[0] else None


def _append_series(store, series_e):
    for k, v in series_e.items():
        store.setdefault(k, []).append(v)


def _collect_ts(series, envs, conds, stacked, e, cond_id, joint_names):
    """Add one env's standard signals (flattened to columns) for parquet."""
    ev = _env_view(stacked, e)
    # NOTE: root_* = root_link origin (at foot level for Chaos URDF), NOT body CoM.
    flat = {
        "root_x": ev["root_pos"][:, 0], "root_y": ev["root_pos"][:, 1], "root_z": ev["root_pos"][:, 2],
        "vbx": ev["root_lin_vel_b"][:, 0], "vby": ev["root_lin_vel_b"][:, 1],
        "tilt": ev["tilt"],
        "lfoot_z": ev["foot_pos"][:, 0, 2], "rfoot_z": ev["foot_pos"][:, 1, 2],
        "lfoot_contact": (ev["foot_contact"][:, 0] > C.CONTACT_FORCE_THRESHOLD).astype(float),
        "rfoot_contact": (ev["foot_contact"][:, 1] > C.CONTACT_FORCE_THRESHOLD).astype(float),
        "cmd_vx": ev["command"][:, 0], "cmd_vy": ev["command"][:, 1],
    }
    for j, nm in enumerate(joint_names):
        flat[f"q_{nm}"] = ev["joint_pos"][:, j]
    for k, v in flat.items():
        series.setdefault(k, []).append(v)
    envs.append(e)
    conds.append(cond_id)
