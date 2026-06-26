# Copyright (c) 2025-2026, The RoboLab Project Developers.
# SPDX-License-Identifier: BSD-3-Clause
"""Pure metric functions for AMP evaluation.

No Isaac/torch dependency -- everything operates on numpy arrays so it can be
unit-tested offline. Shape conventions per *single env* trajectory:
  - scalar-per-step signals: (T,)
  - per-joint signals:       (T, J)
  - foot position:           (T, 2) horizontal or (T,) vertical
Aggregation across envs is done by the caller (stack -> nanmean/nanstd).
"""

from __future__ import annotations

import numpy as np

NAN = float("nan")


# --------------------------------------------------------------------------- #
# Basic error metrics
# --------------------------------------------------------------------------- #
def rmse(a: np.ndarray, b: np.ndarray, axis: int = 0) -> np.ndarray:
    a, b = np.asarray(a, float), np.asarray(b, float)
    return np.sqrt(np.mean((a - b) ** 2, axis=axis))


def mae(a: np.ndarray, b: np.ndarray, axis: int = 0) -> np.ndarray:
    a, b = np.asarray(a, float), np.asarray(b, float)
    return np.mean(np.abs(a - b), axis=axis)


def pearson(a: np.ndarray, b: np.ndarray) -> float:
    """Pearson correlation of two 1-D signals; nan if either is constant."""
    a, b = np.asarray(a, float).ravel(), np.asarray(b, float).ravel()
    if a.size < 2 or b.size < 2:
        return NAN
    a = a - a.mean()
    b = b - b.mean()
    denom = np.sqrt((a * a).sum() * (b * b).sum())
    return float((a * b).sum() / denom) if denom > 1e-12 else NAN


# --------------------------------------------------------------------------- #
# Gait-cycle detection (from a per-foot contact boolean signal)
# --------------------------------------------------------------------------- #
def heel_strikes(contact: np.ndarray, min_gap: int = 3) -> np.ndarray:
    """Indices of rising edges (swing -> stance) = heel strikes.

    `min_gap` suppresses chatter by requiring that many steps between strikes.
    """
    c = np.asarray(contact, float) > 0.5
    if c.size < 2:
        return np.empty(0, int)
    rising = np.flatnonzero((~c[:-1]) & c[1:]) + 1
    if rising.size == 0:
        return rising
    kept = [rising[0]]
    for idx in rising[1:]:
        if idx - kept[-1] >= min_gap:
            kept.append(idx)
    return np.asarray(kept, int)


def gait_cycles(strikes: np.ndarray) -> list[tuple[int, int]]:
    """Consecutive [start, end) intervals between same-foot heel strikes."""
    return [(int(strikes[i]), int(strikes[i + 1])) for i in range(len(strikes) - 1)]


# --------------------------------------------------------------------------- #
# Phase normalization (the correct comparison for random_fetch AMP)
# --------------------------------------------------------------------------- #
def phase_average(signal: np.ndarray, strikes: np.ndarray, n_phase: int = 100) -> tuple[np.ndarray, int]:
    """Resample each gait cycle onto a common phase grid [0,1) and average.

    Returns (mean_over_cycles, n_cycles). `signal` may be (T,) or (T, J).
    If fewer than 2 cycles are available returns an all-nan array.
    """
    signal = np.asarray(signal, float)
    cyc = gait_cycles(strikes)
    feat_shape = signal.shape[1:]
    if len(cyc) < 1:
        return np.full((n_phase, *feat_shape), NAN), 0
    phase = np.linspace(0.0, 1.0, n_phase, endpoint=False)
    resampled = []
    for s, e in cyc:
        if e - s < 3:
            continue
        seg = signal[s:e]
        src = np.linspace(0.0, 1.0, seg.shape[0], endpoint=False)
        if seg.ndim == 1:
            resampled.append(np.interp(phase, src, seg))
        else:
            cols = [np.interp(phase, src, seg[:, j]) for j in range(seg.shape[1])]
            resampled.append(np.stack(cols, axis=-1))
    if not resampled:
        return np.full((n_phase, *feat_shape), NAN), 0
    return np.mean(np.stack(resampled, axis=0), axis=0), len(resampled)


def extract_consecutive_cycles(
    signal: np.ndarray,
    strikes: np.ndarray,
    n_cycles: int = 5,
    n_phase: int = 100,
    skip_first: int = 2,
) -> tuple[np.ndarray, int] | tuple[None, int]:
    """Extract n_cycles real consecutive gait cycles from the middle of the episode.

    Each cycle is resampled to n_phase points then concatenated, giving shape
    (n_cycles * n_phase, J).  skip_first skips the first few cycles (startup
    transient).  Returns (array, n_extracted) or (None, 0) if not enough cycles.
    """
    signal = np.asarray(signal, float)
    cyc = gait_cycles(strikes)
    cyc = cyc[skip_first:]          # drop startup transient
    if len(cyc) < n_cycles:
        return None, 0
    # Take cycles from the middle
    start = (len(cyc) - n_cycles) // 2
    selected = cyc[start: start + n_cycles]
    phase = np.linspace(0.0, 1.0, n_phase, endpoint=False)
    feat_shape = signal.shape[1:]
    segments = []
    for s, e in selected:
        if e - s < 3:
            continue
        seg = signal[s:e]
        src = np.linspace(0.0, 1.0, seg.shape[0], endpoint=False)
        if seg.ndim == 1:
            segments.append(np.interp(phase, src, seg))
        else:
            cols = [np.interp(phase, src, seg[:, j]) for j in range(seg.shape[1])]
            segments.append(np.stack(cols, axis=-1))
    if not segments:
        return None, 0
    return np.concatenate(segments, axis=0), len(segments)  # (n_cycles*n_phase, J)


def phase_aligned_tracking(
    learned: np.ndarray, reference: np.ndarray, strikes_learned: np.ndarray,
    strikes_ref: np.ndarray, n_phase: int = 100,
) -> dict:
    """Phase-normalize both signals then compare per joint.

    learned/reference: (T, J) joint trajectories (need not be time-aligned).
    Returns dict with per-joint rmse, corr and the averaged cycles for plotting.
    """
    lp, ln = phase_average(learned, strikes_learned, n_phase)
    rp, rn = phase_average(reference, strikes_ref, n_phase)
    J = lp.shape[1] if lp.ndim == 2 else 1
    lp = lp.reshape(n_phase, J)
    rp = rp.reshape(n_phase, J)
    per_joint_rmse = rmse(lp, rp, axis=0)              # (J,)
    per_joint_corr = np.array([pearson(lp[:, j], rp[:, j]) for j in range(J)])
    return {
        "per_joint_rmse": per_joint_rmse,
        "per_joint_corr": per_joint_corr,
        "learned_cycle": lp,
        "reference_cycle": rp,
        "n_cycles_learned": ln,
        "n_cycles_ref": rn,
    }


def phase_error_lr(left_contact: np.ndarray, right_contact: np.ndarray, fps: float) -> dict:
    """Left/right phase offset as a fraction of the stride (ideal 0.5)."""
    ls = heel_strikes(left_contact)
    rs = heel_strikes(right_contact)
    if ls.size < 2 or rs.size < 1:
        return {"lag_steps": NAN, "lag_fraction": NAN}
    stride = float(np.mean(np.diff(ls)))               # steps per left stride
    # For each left strike, distance to the next right strike.
    fracs = []
    for l in ls:
        later = rs[rs > l]
        if later.size:
            fracs.append((later[0] - l) / stride)
    if not fracs:
        return {"lag_steps": NAN, "lag_fraction": NAN}
    frac = float(np.mean(fracs))
    return {"lag_steps": frac * stride, "lag_fraction": frac}


# --------------------------------------------------------------------------- #
# Gait quality
# --------------------------------------------------------------------------- #
def stride_length(foot_xy: np.ndarray, strikes: np.ndarray) -> float:
    """Mean horizontal distance between successive same-foot heel strikes."""
    if strikes.size < 2:
        return NAN
    pts = np.asarray(foot_xy, float)[strikes]
    return float(np.mean(np.linalg.norm(np.diff(pts, axis=0), axis=1)))


def step_frequency(left_contact: np.ndarray, right_contact: np.ndarray, fps: float) -> float:
    """Steps per second = total heel strikes / duration."""
    n = heel_strikes(left_contact).size + heel_strikes(right_contact).size
    T = max(len(left_contact), 1)
    dur = T / fps
    return float(n / dur) if dur > 0 else NAN


def duty_factor(contact: np.ndarray) -> float:
    """Fraction of time the foot is in stance."""
    c = np.asarray(contact, float) > 0.5
    return float(c.mean()) if c.size else NAN


def double_support_ratio(left_contact: np.ndarray, right_contact: np.ndarray) -> float:
    """Fraction of time both feet are in contact."""
    l = np.asarray(left_contact, float) > 0.5
    r = np.asarray(right_contact, float) > 0.5
    return float((l & r).mean()) if l.size else NAN


def symmetry_index(l_value: float, r_value: float) -> float:
    """|L-R| / (0.5(L+R)); 0 = perfectly symmetric."""
    if np.isnan(l_value) or np.isnan(r_value):
        return NAN
    denom = 0.5 * (abs(l_value) + abs(r_value))
    return float(abs(l_value - r_value) / denom) if denom > 1e-9 else NAN


def foot_clearance(foot_z: np.ndarray, contact: np.ndarray) -> float:
    """Max foot height during swing (contact == 0), relative to stance min."""
    z = np.asarray(foot_z, float)
    c = np.asarray(contact, float) > 0.5
    if (~c).sum() == 0 or c.sum() == 0:
        return NAN
    ground = float(np.median(z[c]))
    return float(z[~c].max() - ground)


def foot_sliding(foot_vel_xy: np.ndarray, contact: np.ndarray) -> float:
    """Mean horizontal foot speed while in contact (the AMP sliding metric)."""
    v = np.linalg.norm(np.asarray(foot_vel_xy, float), axis=-1)
    c = np.asarray(contact, float) > 0.5
    return float(v[c].mean()) if c.sum() else NAN


# --------------------------------------------------------------------------- #
# CoM stability
# --------------------------------------------------------------------------- #
def com_stability(com_pos: np.ndarray, com_vel: np.ndarray, fps: float) -> dict:
    """com_pos/com_vel: (T, 3) world. Lateral deviation around mean y."""
    p = np.asarray(com_pos, float)
    v = np.asarray(com_vel, float)
    accel = np.diff(v, axis=0) * fps if v.shape[0] > 1 else np.zeros((1, 3))
    return {
        "height_mean": float(p[:, 2].mean()),
        "height_std": float(p[:, 2].std()),
        "lateral_deviation_max": float(np.abs(p[:, 1] - p[:, 1].mean()).max()),
        "velocity_smoothness": float(np.mean(np.linalg.norm(accel, axis=1))),
    }


# --------------------------------------------------------------------------- #
# Energy
# --------------------------------------------------------------------------- #
def total_joint_energy(tau: np.ndarray, omega: np.ndarray, dt: float) -> float:
    """Sum_t Sum_j |tau*omega| * dt  (mechanical energy magnitude, J)."""
    p = np.abs(np.asarray(tau, float) * np.asarray(omega, float))
    return float(p.sum() * dt)


def cost_of_transport(energy_j: float, mass: float, distance_m: float) -> float:
    if distance_m <= 1e-6 or mass <= 0:
        return NAN
    return float(energy_j / (mass * 9.81 * distance_m))


def power_per_joint(tau: np.ndarray, omega: np.ndarray) -> np.ndarray:
    return np.mean(np.abs(np.asarray(tau, float) * np.asarray(omega, float)), axis=0)


def specific_resistance(mean_power_w: float, mass: float, speed: float) -> float:
    if speed <= 1e-6 or mass <= 0:
        return NAN
    return float(mean_power_w / (mass * 9.81 * speed))


# --------------------------------------------------------------------------- #
# Velocity tracking + step response
# --------------------------------------------------------------------------- #
def velocity_tracking(v_actual: np.ndarray, v_cmd: np.ndarray) -> dict:
    a = np.asarray(v_actual, float)
    c = np.asarray(v_cmd, float)
    err = a - c
    n_tail = max(1, len(a) // 5)
    return {
        "rmse": float(np.sqrt(np.mean(err ** 2))),
        "mae": float(np.mean(np.abs(err))),
        "steady_state_error": float(np.mean(np.abs(err[-n_tail:]))),
    }


def step_response_metrics(signal: np.ndarray, fps: float, v0: float, v_target: float,
                          settle_band: float = 0.05) -> dict:
    """Rise/settling/overshoot for a 1-D response to a command step v0 -> v_target.

    `signal` starts at the moment of the step. settle_band is fraction of the
    step size used for the 5% settling criterion.
    """
    s = np.asarray(signal, float)
    delta = v_target - v0
    t = np.arange(s.size) / fps
    if abs(delta) < 1e-6 or s.size == 0:
        return {"rise_time_s": NAN, "settling_time_s": NAN, "overshoot_pct": NAN}

    # Rise time: first crossing of 90% of the step.
    target_90 = v0 + 0.9 * delta
    crossed = (s >= target_90) if delta > 0 else (s <= target_90)
    rise = float(t[np.argmax(crossed)]) if crossed.any() else NAN

    # Settling time: last time the error leaves the +-band around v_target.
    band = abs(settle_band * delta)
    outside = np.abs(s - v_target) > band
    settle = float(t[np.max(np.flatnonzero(outside)) + 1]) if outside.any() and np.flatnonzero(outside)[-1] + 1 < s.size else (0.0 if not outside.any() else NAN)

    # Overshoot relative to target.
    peak = s.max() if delta > 0 else s.min()
    overshoot = float((peak - v_target) / delta * 100.0)
    overshoot = max(overshoot, 0.0)
    return {"rise_time_s": rise, "settling_time_s": settle, "overshoot_pct": overshoot}


# --------------------------------------------------------------------------- #
# Survival / fall (per env, over a window)
# --------------------------------------------------------------------------- #
def fall_mask(base_z: np.ndarray, tilt_rad: np.ndarray, height_thr: float, tilt_thr: float) -> np.ndarray:
    """Boolean per-step: True where the robot is considered fallen."""
    z = np.asarray(base_z, float)
    a = np.asarray(tilt_rad, float)
    return (z < height_thr) | (a > tilt_thr)


def survived(base_z: np.ndarray, tilt_rad: np.ndarray, height_thr: float, tilt_thr: float) -> bool:
    """True if the robot never falls within the provided window."""
    return not bool(fall_mask(base_z, tilt_rad, height_thr, tilt_thr).any())


def recovery_time(com_err: np.ndarray, fps: float, tol: float) -> float:
    """Steps until CoM error falls below tol and stays, converted to seconds."""
    e = np.asarray(com_err, float)
    bad = e > tol
    if not bad.any():
        return 0.0
    last_bad = int(np.max(np.flatnonzero(bad)))
    return NAN if last_bad + 1 >= e.size else float((last_bad + 1) / fps)


# --------------------------------------------------------------------------- #
# Aggregation helper
# --------------------------------------------------------------------------- #
def agg(values: list[float]) -> dict:
    """mean/std/n over a list, ignoring nan."""
    arr = np.asarray(values, float)
    arr = arr[~np.isnan(arr)]
    if arr.size == 0:
        return {"mean": NAN, "std": NAN, "n": 0}
    return {"mean": float(arr.mean()), "std": float(arr.std()), "n": int(arr.size)}
