# Copyright (c) 2025-2026, The RoboLab Project Developers.
# SPDX-License-Identifier: BSD-3-Clause
"""Render PNG charts from the evaluation summary JSON dicts.

Kept separate from metric computation so plots can be regenerated any time from
the saved tables. All functions no-op gracefully if matplotlib is missing.
"""

from __future__ import annotations

import os

import numpy as np

try:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    _HAVE_MPL = True
except Exception:  # pragma: no cover
    _HAVE_MPL = False


def _save(fig, path: str) -> None:
    os.makedirs(os.path.dirname(path), exist_ok=True)
    fig.savefig(path, dpi=130, bbox_inches="tight")
    plt.close(fig)
    print(f"[eval] plot -> {path}")


# --------------------------------------------------------------------------- #
# Phase 2: joint angle vs gait phase (learned vs reference)
# --------------------------------------------------------------------------- #
def plot_motion_quality(motion_json: dict, out_dir: str) -> None:
    if not _HAVE_MPL:
        return
    for run in motion_json.get("runs", []):
        cycles = run.get("cycles")
        if not cycles:
            continue
        learned = np.asarray(cycles["learned"])      # (n_cycles*P, J) - real consecutive cycles
        ref     = np.asarray(cycles["reference"])    # (n_cycles*P, J) - reference tiled to match
        names = cycles.get("joint_names", [f"j{j}" for j in range(learned.shape[1])])
        n_cycles   = cycles.get("n_cycles", 1)
        n_phase_pt = cycles.get("n_phase_pts", learned.shape[0] // max(n_cycles, 1))
        J = learned.shape[1]
        n_survived = run.get("n_envs_survived", "?")

        # X axis: each integer = 1 complete gait cycle
        total_pts = learned.shape[0]
        phase = np.linspace(0, n_cycles, total_pts, endpoint=False)

        ncol = 3
        nrow = int(np.ceil(J / ncol))
        fig, axes = plt.subplots(nrow, ncol, figsize=(4 * ncol, 2.4 * nrow), squeeze=False)
        for j in range(J):
            ax = axes[j // ncol][j % ncol]
            ax.plot(phase, learned[:, j], color="#3399ff", lw=1.8, label="learned (sim)")
            ax.plot(phase, ref[:, j],     color="#ff6633", ls="--", lw=1.5, label="reference (clip)")
            for c in range(1, n_cycles):
                ax.axvline(c, color="grey", lw=0.6, ls=":", alpha=0.5)
            ax.set_title(names[j], fontsize=8)
            ax.set_xlabel("Gait cycle (1 unit = 1 complete cycle)", fontsize=7)
            ax.set_ylabel("joint angle [rad]", fontsize=7)
            ax.set_xticks(range(n_cycles + 1))
            ax.tick_params(labelsize=7)
            ax.grid(alpha=0.25)
        for k in range(J, nrow * ncol):
            axes[k // ncol][k % ncol].axis("off")
        axes[0][0].legend(fontsize=7, loc="upper right")
        fig.suptitle(
            f"Phase 2 - Joint angle vs gait cycle  |  clip={run['clip']}  "
            f"cmd_vx={run['cmd_vx']} m/s  |  {n_survived} env survived\n"
            f"{n_cycles} real consecutive cycles from one representative env "
            f"({n_phase_pt} pts/cycle, sampled from mid-episode to avoid initial transients)",
            fontsize=10,
        )
        fig.tight_layout(rect=(0, 0, 1, 0.95))
        _save(fig, os.path.join(out_dir, f"motion_phase_{run['clip']}.png"))


# --------------------------------------------------------------------------- #
# Phase 3: performance curves vs velocity
# --------------------------------------------------------------------------- #
def plot_velocity_sweep(task_json: dict, out_dir: str) -> None:
    """Phase 3 - performance vs commanded velocity (per axis sweep)."""
    if not _HAVE_MPL:
        return
    table = task_json.get("velocity_sweep", {}).get("table", [])
    if not table:
        return
    metrics = [
        ("vel_rmse", "Velocity RMSE [m/s]\n(lower = better tracking)"),
        ("cot", "Cost of Transport [-]\n(human walk approx. 1.4)"),
        ("survival", "Survival rate\n(fraction of envs not fallen)"),
        ("foot_sliding_ms", "Foot sliding [m/s]\n(threshold 0.01)"),
    ]
    axis_info = {
        "vx": ("cmd_vx", "Commanded forward velocity vx [m/s]"),
        "vy": ("cmd_vy", "Commanded lateral velocity vy [m/s]"),
        "wz": ("cmd_wz", "Commanded yaw rate wz [rad/s]"),
    }
    for axis, (col_key, xlabel) in axis_info.items():
        rows = [r for r in table if r.get("axis") == axis]
        if not rows:
            continue
        rows = sorted(rows, key=lambda r: r[col_key])
        x = [r[col_key] for r in rows]
        in_ds = [r.get("in_dataset_range", False) for r in rows]
        fig, axes = plt.subplots(2, 2, figsize=(11, 7.5))
        for ax, (key, ylabel) in zip(axes.ravel(), metrics):
            y = [r.get(key, np.nan) for r in rows]
            ax.plot(x, y, "-o", color="#3399ff", lw=1.5, ms=6,
                    label="extrapolation (outside dataset)")
            for xi, yi, ds in zip(x, y, in_ds):
                if ds:
                    ax.plot(xi, yi, "o", color="#ff6633", ms=11, zorder=5,
                            label="_in_dataset")
            # one labelled orange marker just for the legend
            ds_pts = [(xi, yi) for xi, yi, d in zip(x, y, in_ds) if d]
            if ds_pts:
                ax.plot(*ds_pts[0], "o", color="#ff6633", ms=11, zorder=5,
                        label="speed present in AMP dataset")
            ax.axvline(0, color="grey", lw=0.5, alpha=0.5)
            ax.set_xlabel(xlabel, fontsize=9)
            ax.set_ylabel(ylabel, fontsize=9)
            ax.grid(alpha=0.3)
            ax.legend(fontsize=7, loc="best")
        fig.suptitle(
            f"Phase 3 - Velocity sweep along {axis} axis "
            f"(other 2 cmd axes held at 0)\n"
            f"Blue line = policy at each commanded speed; orange = clip-mean speeds in training set",
            fontsize=11,
        )
        fig.tight_layout(rect=(0, 0, 1, 0.94))
        _save(fig, os.path.join(out_dir, f"sweep_{axis}.png"))


# --------------------------------------------------------------------------- #
# Phase 4: push robustness
# --------------------------------------------------------------------------- #
def plot_push_robustness(robust_json: dict, out_dir: str) -> None:
    if not _HAVE_MPL:
        return
    push = robust_json.get("push_robustness", {})
    table = push.get("table", [])
    if not table:
        return
    directions = push.get("directions", ["forward", "backward", "left", "right"])
    dir_styles = {
        "forward":  {"color": "#3399ff", "ls": "-",  "marker": "o"},
        "backward": {"color": "#ff6633", "ls": "--", "marker": "s"},
        "left":     {"color": "#33aa55", "ls": "-.", "marker": "^"},
        "right":    {"color": "#cc44cc", "ls": ":",  "marker": "D"},
    }
    cmd_levels = sorted({r["cmd_vx"] for r in table})
    fig, axes = plt.subplots(1, len(cmd_levels), figsize=(5 * len(cmd_levels), 4.5), squeeze=False)
    for ax, cmd in zip(axes[0], cmd_levels):
        rows = sorted([r for r in table if r["cmd_vx"] == cmd], key=lambda r: r["push_vel_ms"])
        x = [r["push_vel_ms"] for r in rows]
        for d in directions:
            st = dir_styles.get(d, {})
            ax.plot(x, [r.get(d, np.nan) for r in rows],
                    ls=st.get("ls", "-"), marker=st.get("marker", "o"),
                    color=st.get("color", None), lw=1.8, ms=7, label=d)
        ax.axhline(0.5, color="grey", ls=":", lw=1, label="50% threshold")
        ax.set_title(f"cmd_vx = {cmd} m/s  (vy=0, wz=0)", fontsize=10)
        ax.set_xlabel("Push velocity magnitude [m/s]\n(root velocity impulse applied for 1 step)", fontsize=9)
        ax.set_ylabel("Survival rate\n(fraction of envs not fallen after 3s)", fontsize=9)
        ax.set_ylim(-0.05, 1.05)
        ax.grid(alpha=0.3)
        ax.legend(fontsize=8, loc="upper right")
    fig.suptitle(
        "Phase 4 - Push robustness: survival rate vs push magnitude\n"
        "Each point = fraction of envs still upright 3s after a 1-step velocity impulse",
        fontsize=11,
    )
    _save(fig, os.path.join(out_dir, "push_robustness.png"))


def plot_noise_robustness(robust_json: dict, out_dir: str) -> None:
    if not _HAVE_MPL:
        return
    table = robust_json.get("noise_robustness", {}).get("table", [])
    if not table:
        return
    rows = sorted(table, key=lambda r: r["sigma"])
    x = [r["sigma"] for r in rows]
    fig, ax1 = plt.subplots(figsize=(7, 4.5))
    ax1.plot(x, [r["survival"] for r in rows], "-o", color="#33aa55", label="survival")
    ax1.set_xlabel("observation noise sigma")
    ax1.set_ylabel("survival", color="#33aa55")
    ax1.set_ylim(-0.05, 1.05)
    ax2 = ax1.twinx()
    ax2.plot(x, [r.get("vel_rmse", np.nan) for r in rows], "-s", color="#3399ff", label="vel rmse")
    ax2.set_ylabel("velocity RMSE [m/s]", color="#3399ff")
    fig.suptitle("Noise robustness", fontsize=11)
    _save(fig, os.path.join(out_dir, "noise_robustness.png"))


def plot_all(out_dir: str, motion=None, task=None, robust=None) -> None:
    if motion:
        plot_motion_quality(motion, out_dir)
    if task:
        plot_velocity_sweep(task, out_dir)
    if robust:
        plot_push_robustness(robust, out_dir)
        plot_noise_robustness(robust, out_dir)
