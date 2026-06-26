"""Standalone Isaac Sim script: spawn robot Chaos o POSE DEFAULT (chi xem, khong test).

- Dung CHAOS_CFG (floating base) — dung cau hinh nhu trong chaos.py
- Pose = init_state.joint_pos da cfg trong chaos.py
- Co ground plane + dome light; GUI mo san de quan sat
- Vong lap giu joint target = default pose (PD giu tu the het muc co the)

Cach chay:
    python scripts/tools/spawn_default.py

Dong cua so GUI de thoat.
"""

import argparse

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser(description="Spawn Chaos robot o pose default de xem")
parser.add_argument("--fixed", action="store_true",
                    help="Giu base co dinh (mac dinh: tha tu do de roi duoi gravity)")
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()
app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

# --- Isaac Sim imports sau AppLauncher ---
import omni.usd
import isaaclab.sim as sim_utils
from isaaclab.sim import SimulationContext
from isaaclab.assets import Articulation

from robolab.assets.robots.chaos import CHAOS_CFG

PHYSICS_DT = 0.005  # 200 Hz


def main():
    # --- Setup sim: gravity ON ---
    sim = SimulationContext(sim_utils.SimulationCfg(dt=PHYSICS_DT, device="cpu"))
    sim.set_camera_view(eye=[1.5, 1.5, 1.0], target=[0.0, 0.0, 0.5])

    # --- Ground plane + light ---
    ground_cfg = sim_utils.GroundPlaneCfg()
    ground_cfg.func("/World/ground", ground_cfg)
    light_cfg = sim_utils.DomeLightCfg(intensity=2500.0, color=(0.9, 0.9, 0.9))
    light_cfg.func("/World/light", light_cfg)

    # --- Spawn robot USD truoc (de phau thuat truoc khi tao Articulation view) ---
    spawn_cfg = CHAOS_CFG.spawn
    spawn_cfg.func("/World/robot", spawn_cfg)

    # test6.usd dat ArticulationRootAPI tren 'root_joint' (PhysicsFixedJoint han
    # base_link vao world) => base CO DINH. De base tu do (roi/nga duoi gravity):
    #   1) xoa root_joint  2) chuyen ArticulationRootAPI sang base_link
    if not args_cli.fixed:
        from pxr import UsdPhysics, PhysxSchema
        stage = omni.usd.get_context().get_stage()
        root = "/World/robot/assem_symplified_6"
        rj = stage.GetPrimAtPath(root + "/root_joint")
        base = stage.GetPrimAtPath(root + "/base_link")
        if rj.IsValid() and base.IsValid():
            # 1) Go ArticulationRootAPI khoi fixed joint + tat rang buoc
            rj.RemoveAPI(UsdPhysics.ArticulationRootAPI)
            rj.RemoveAPI(PhysxSchema.PhysxArticulationAPI)
            if rj.GetAttribute("physics:jointEnabled").IsValid():
                rj.GetAttribute("physics:jointEnabled").Set(False)
            # 2) Dat articulation root len base_link => floating base
            UsdPhysics.ArticulationRootAPI.Apply(base)
            PhysxSchema.PhysxArticulationAPI.Apply(base)
            print("[INFO] root_joint disabled + ArticulationRoot -> base_link (floating)")
        else:
            print("[WARN] Khong tim thay root_joint/base_link — base co the van bi neo")

    # --- Tao Articulation view tren prim co san (khong re-spawn) ---
    cfg = CHAOS_CFG.replace(prim_path="/World/robot", spawn=None)
    robot = Articulation(cfg)

    sim.reset()

    # --- Dat robot dung init_state (pos z, joint_pos) tu chaos.py xuong sim ---
    # Neu KHONG ghi root pose, robot spawn o vi tri mac dinh cua USD (~goc) chu
    # khong phai init_state.pos => khong thay no roi tu z da dinh nghia.
    root_state = robot.data.default_root_state.clone()
    robot.write_root_pose_to_sim(root_state[:, :7])       # pos(3) + quat(4)
    robot.write_root_velocity_to_sim(root_state[:, 7:])   # lin(3) + ang(3)
    robot.write_joint_state_to_sim(
        robot.data.default_joint_pos.clone(), robot.data.default_joint_vel.clone()
    )
    robot.reset()

    # --- Pose default tu init_state.joint_pos (da nap vao default_joint_pos) ---
    default_pos = robot.data.default_joint_pos.clone()

    print(f"\nAll joints: {list(enumerate(robot.joint_names))}")
    print("Default joint pos (rad):")
    for name, q in zip(robot.joint_names, default_pos[0].tolist()):
        print(f"  {name:<26} {q:+.4f}")
    print("\nDang giu pose default. Dong cua so GUI de thoat.\n")

    # --- Vong lap: giu joint target = default pose, base roi tu do duoi gravity ---
    step = 0
    while simulation_app.is_running():
        robot.set_joint_position_target(default_pos)
        robot.write_data_to_sim()
        sim.step()
        robot.update(PHYSICS_DT)

        if step % 100 == 0:  # ~moi 0.5s in chieu cao base
            z = robot.data.root_pos_w[0, 2].item()
            print(f"  t={step*PHYSICS_DT:5.2f}s  base_z={z:+.4f} m")
        step += 1

    simulation_app.close()


if __name__ == "__main__":
    main()
