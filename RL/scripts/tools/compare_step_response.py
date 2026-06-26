"""So sánh step response sim vs real robot.

Cách chạy (không cần Isaac Sim):
    python scripts/tools/compare_step_response.py \
        --real_dir  "C:/Users/ngoc/Downloads/drive-download-20260522T054649Z-3-001" \
        --sim_dir   "scripts/tools/step_response_data" \
        --joint     right_knee_joint \
        --targets   5 10 15 25 35 45 55 65
"""

import argparse
import json
import math
import os
import zipfile
import xml.etree.ElementTree as ET
import matplotlib.pyplot as plt

parser = argparse.ArgumentParser()
parser.add_argument("--real_dir",  type=str, required=True)
parser.add_argument("--sim_dir",   type=str, required=True)
parser.add_argument("--joint",     type=str, default="right_knee_joint")
parser.add_argument("--targets",   type=float, nargs="+",
                    default=[5, 10, 15, 25, 35, 45, 55, 65])
parser.add_argument("--out",       type=str, default="step_response_compare.png")
args = parser.parse_args()

ns = {"s": "http://schemas.openxmlformats.org/spreadsheetml/2006/main"}

def read_real(path):
    """Đọc xlsx real robot → list of (t_ms, pos_deg, vel_rpm)."""
    with zipfile.ZipFile(path) as z:
        tree = ET.parse(z.open("xl/worksheets/sheet1.xml"))
    root = tree.getroot()
    data = []
    for row in root.findall(".//s:row", ns):
        cells = [c.find("s:v", ns) for c in row.findall("s:c", ns)]
        vals  = [float(c.text) if c is not None else None for c in cells]
        if len(vals) >= 4 and vals[0] is not None:
            data.append((vals[1], vals[2], vals[3]))  # t_ms, pos_deg, vel_rpm
    return data

def read_sim(path):
    """Đọc json sim → list of (t_ms, pos_deg, vel_rpm)."""
    with open(path) as f:
        d = json.load(f)
    return [(e["t"], e["pos_deg"], e["vel_rpm"]) for e in d["log"]]


colors = ["#e41a1c","#377eb8","#4daf4a","#984ea3","#ff7f00","#a65628","#f781bf","#555555"]
SIM_VEL_LIMIT = 6.981  # rad/s
RADS2RPM = 60 / (2 * math.pi)

n = len(args.targets)
fig, axes = plt.subplots(n, 2, figsize=(14, 3 * n))
if n == 1:
    axes = [axes]

print(f"\n{'Target':>8} | {'Real final':>10} | {'Sim final':>10} | {'Pos gap':>9} | {'Real maxRPM':>12} | {'Sim maxRPM':>11}")
print("-" * 70)

for i, (target, color) in enumerate(zip(args.targets, colors)):
    real_path = os.path.join(args.real_dir, f"test_sim_{int(target)}.xlsx")
    sim_path  = os.path.join(args.sim_dir,
                             f"step_response_sim_{args.joint}_{int(target)}deg.json")

    ax_pos = axes[i][0]
    ax_vel = axes[i][1]

    if os.path.exists(real_path):
        real = read_real(real_path)
        t_r   = [r[0] for r in real]
        pos_r = [r[1] for r in real]
        rpm_r = [r[2] for r in real]
        ax_pos.plot(t_r, pos_r, color=color, linewidth=2, label="real")
        ax_vel.plot(t_r, rpm_r, color=color, linewidth=2, label="real")
        real_final  = pos_r[-1]
        real_maxrpm = max(rpm_r)
    else:
        real_final = real_maxrpm = float("nan")
        print(f"  WARNING: real data not found: {real_path}")

    if os.path.exists(sim_path):
        sim = read_sim(sim_path)
        t_s   = [r[0] for r in sim]
        pos_s = [r[1] for r in sim]
        rpm_s = [r[2] for r in sim]
        ax_pos.plot(t_s, pos_s, color=color, linewidth=2, linestyle="--", label="sim")
        ax_vel.plot(t_s, rpm_s, color=color, linewidth=2, linestyle="--", label="sim")
        sim_final  = pos_s[-1]
        sim_maxrpm = max(rpm_s)
    else:
        sim_final = sim_maxrpm = float("nan")
        print(f"  WARNING: sim data not found: {sim_path}")

    ax_pos.axhline(target, color="gray", linestyle=":", linewidth=1, label=f"target={target}°")
    ax_pos.set_ylabel("Position (deg)")
    ax_pos.set_title(f"target={target}° — Position")
    ax_pos.legend(fontsize=8); ax_pos.grid(True, alpha=0.4)

    ax_vel.axhline(SIM_VEL_LIMIT * RADS2RPM, color="black", linestyle="--",
                   linewidth=1.2, label=f"sim limit={SIM_VEL_LIMIT:.2f} rad/s")
    ax_vel.set_ylabel("Velocity (RPM)")
    ax_vel.set_title(f"target={target}° — Velocity")
    ax_vel.legend(fontsize=8); ax_vel.grid(True, alpha=0.4)

    if i == n - 1:
        ax_pos.set_xlabel("Time (ms)")
        ax_vel.set_xlabel("Time (ms)")

    gap = abs(real_final - sim_final) if not math.isnan(real_final + sim_final) else float("nan")
    print(f"{target:>7}° | {real_final:>10.3f} | {sim_final:>10.3f} | {gap:>9.3f} | "
          f"{real_maxrpm:>12.1f} | {sim_maxrpm:>11.1f}")

plt.suptitle(f"Step Response: Sim vs Real — {args.joint}", fontsize=13, fontweight="bold")
plt.tight_layout()
plt.savefig(args.out, dpi=150)
print(f"\nSaved: {args.out}")
