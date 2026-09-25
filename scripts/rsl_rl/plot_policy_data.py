"""Plot policy data printed by scripts/rsl_rl/play.py --print_policy_data.

The policy observation is history-flattened term-by-term in the env config. This
script reconstructs the newest 47-dim frame before applying the requested
per-frame indexes:

    0:6   first six policy observations
    9:21  joint positions
    21:33 joint velocities
    action_scaled[0:12] joint actions
"""

from __future__ import annotations

import argparse
import ast
import re
from pathlib import Path

import matplotlib.pyplot as plt
from matplotlib.backends.backend_pdf import PdfPages


POLICY_RE = re.compile(
    r"^\[POLICY\]\s+step=(?P<step>\d+)\s*\n"
    r"obs_policy=(?P<obs_policy>.*?)\n"
    r"obs_critic_first45=.*?\n"
    r"actions_raw=.*?\n"
    r"actions_scaled=(?P<actions_scaled>.*?)(?=\n\[POLICY\]|\n\[RTF\]|\Z)",
    re.MULTILINE | re.DOTALL,
)

TERM_DIMS = (3, 3, 3, 12, 12, 12, 2)
FRAME_DIM = sum(TERM_DIMS)


def _first_env(data):
    """Return the first env row from tensors printed as nested Python lists."""
    if isinstance(data, list) and data and isinstance(data[0], list):
        return data[0]
    return data


def _parse_list(text: str) -> list[float]:
    value = ast.literal_eval(text.strip())
    value = _first_env(value)
    return [float(item) for item in value]


def _newest_frame_from_history(obs_policy: list[float], history_length: int) -> list[float]:
    if len(obs_policy) == FRAME_DIM:
        return obs_policy

    expected_dim = FRAME_DIM * history_length
    if len(obs_policy) != expected_dim:
        raise ValueError(
            f"Expected obs_policy length {FRAME_DIM} or {expected_dim}, got {len(obs_policy)}. "
            "Check --history_length or the env observation layout."
        )

    frame: list[float] = []
    offset = 0
    for term_dim in TERM_DIMS:
        term_history = obs_policy[offset : offset + term_dim * history_length]
        frame.extend(term_history[-term_dim:])
        offset += term_dim * history_length
    return frame


def load_policy_records(log_path: Path, history_length: int) -> tuple[list[int], list[list[float]], list[list[float]]]:
    text = log_path.read_text(encoding="utf-8", errors="replace")
    steps: list[int] = []
    obs_frames: list[list[float]] = []
    actions_scaled: list[list[float]] = []

    for match in POLICY_RE.finditer(text):
        obs_policy = _parse_list(match.group("obs_policy"))
        action = _parse_list(match.group("actions_scaled"))
        steps.append(int(match.group("step")))
        obs_frames.append(_newest_frame_from_history(obs_policy, history_length))
        actions_scaled.append(action[:12])

    if not steps:
        raise ValueError(
            "No [POLICY] blocks found. Run play.py with --print_policy_data and redirect output to a text file."
        )
    return steps, obs_frames, actions_scaled


def plot_to_pdf(steps: list[int], obs_frames: list[list[float]], actions_scaled: list[list[float]], output_path: Path):
    output_path.parent.mkdir(parents=True, exist_ok=True)

    first_six = list(zip(*(frame[:6] for frame in obs_frames)))
    joint_pos = list(zip(*(frame[9:21] for frame in obs_frames)))
    joint_vel = list(zip(*(frame[21:33] for frame in obs_frames)))
    actions = list(zip(*actions_scaled))

    with PdfPages(output_path) as pdf:
        fig, ax = plt.subplots(figsize=(11, 6))
        for index, values in enumerate(first_six):
            ax.plot(steps, values, label=f"obs[{index}]")
        ax.set_title("First 6 obs_policy values - newest frame")
        ax.set_xlabel("Step")
        ax.grid(True, alpha=0.3)
        ax.legend(ncol=3)
        fig.tight_layout()
        pdf.savefig(fig)
        plt.close(fig)

        for joint_index in range(12):
            fig, ax = plt.subplots(figsize=(11, 6))
            ax.plot(steps, joint_pos[joint_index], label=f"pos obs[{9 + joint_index}]")
            ax.plot(steps, joint_vel[joint_index], label=f"vel obs[{21 + joint_index}]")
            ax.plot(steps, actions[joint_index], label=f"action_scaled[{joint_index}]")
            ax.set_title(f"Joint {joint_index}: position, velocity, scaled action")
            ax.set_xlabel("Step")
            ax.grid(True, alpha=0.3)
            ax.legend()
            fig.tight_layout()
            pdf.savefig(fig)
            plt.close(fig)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("log_path", type=Path, help="Text file captured from play.py --print_policy_data")
    parser.add_argument(
        "-o",
        "--output",
        type=Path,
        default=Path("policy_data_plots.pdf"),
        help="Output PDF path. Defaults to policy_data_plots.pdf",
    )
    parser.add_argument("--history_length", type=int, default=5, help="Observation history length. Defaults to 5.")
    args = parser.parse_args()

    steps, obs_frames, actions_scaled = load_policy_records(args.log_path, args.history_length)
    plot_to_pdf(steps, obs_frames, actions_scaled, args.output)
    print(f"Saved {len(steps)} samples into {args.output}")


if __name__ == "__main__":
    main()
