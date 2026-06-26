"""Inspect authored masses and centers of mass in a USD file.

Run from the project root, for example:
    conda run -n env_isaaclab python robolab/robolab-732b710f294926b8dd057057a6c3d86634c51e3f/scripts/tools/inspect_usd_masses.py --headless --usd D:/IsaacSim/test6.usd
"""

import argparse
import os
import sys

from isaaclab.app import AppLauncher


parser = argparse.ArgumentParser(description="Print MassAPI values authored in a USD file.")
parser.add_argument("--usd", type=str, default="D:/IsaacSim/test6.usd", help="Path to USD/USDC file")
parser.add_argument("--out", type=str, default="", help="Optional text output path")
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()

app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

from pxr import Usd, UsdPhysics  # noqa: E402


def shutdown(exit_code=0):
    try:
        simulation_app.close()
    finally:
        sys.stdout.flush()
        sys.stderr.flush()
        os._exit(exit_code)


def main():
    stage = Usd.Stage.Open(args_cli.usd)
    if stage is None:
        print(f"ERROR: cannot open USD: {args_cli.usd}")
        shutdown(1)

    rows = []
    for prim in stage.Traverse():
        mass_api = UsdPhysics.MassAPI(prim)
        rigid_body_api = UsdPhysics.RigidBodyAPI(prim)
        mass_attr = mass_api.GetMassAttr()
        com_attr = mass_api.GetCenterOfMassAttr()
        has_mass = mass_attr.HasAuthoredValueOpinion()
        has_com = com_attr.HasAuthoredValueOpinion()
        has_rigid_body = bool(rigid_body_api)
        if has_mass or has_com or has_rigid_body:
            rows.append(
                {
                    "path": prim.GetPath().pathString,
                    "mass": mass_attr.Get() if has_mass else None,
                    "com": com_attr.Get() if has_com else None,
                    "rigid_body": has_rigid_body,
                }
            )

    lines = [
        f"USD: {args_cli.usd}",
        f"Prim count with MassAPI or RigidBodyAPI: {len(rows)}",
        "",
    ]
    total_authored_mass = 0.0
    for row in rows:
        mass = row["mass"]
        if mass is not None:
            total_authored_mass += float(mass)
        mass_text = "None" if mass is None else f"{float(mass):.6g}"
        com = row["com"]
        if com is None:
            com_text = "None"
        else:
            com_text = f"({float(com[0]):.6g}, {float(com[1]):.6g}, {float(com[2]):.6g})"
        lines.append(
            f"{row['path']}\tmass={mass_text}\tcom={com_text}\trigid_body={row['rigid_body']}"
        )

    lines.append("")
    lines.append(f"Total authored mass: {total_authored_mass:.6g}")
    text = "\n".join(lines)
    print(text)
    if args_cli.out:
        with open(args_cli.out, "w", encoding="utf-8") as f:
            f.write(text + "\n")
    shutdown(0)


if __name__ == "__main__":
    main()
