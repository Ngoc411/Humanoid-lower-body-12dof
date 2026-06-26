"""Standalone Isaac Sim script: đo step response từng khớp để so sánh với robot thật.

- Dùng CHAOS_CFG bình thường, không ép fixed-base trong script
- Start position: 0 rad cho tất cả joints (khớp với real robot data)
- Log: Sample, Time Step (ms), Actual Angle (deg), Actual RPM, Torque (Nm)

Cách chạy:
    python scripts/tools/step_response_sim.py --joint right_knee_joint
    python scripts/tools/step_response_sim.py --joint right_knee_joint --obstacle
    python scripts/tools/step_response_sim.py --joint right_knee_joint --no-ground
    python scripts/tools/step_response_sim.py --joint right_knee_joint --obstacle --no-ground

    # Chỉnh vị trí/kích thước box:
    python scripts/tools/step_response_sim.py --obstacle \\
        --obstacle-front  0.15  0.0  0.05 \\
        --obstacle-rear  -0.10  0.0  0.05 \\
        --obstacle-size   0.20  0.12 0.06

    # Toggle real-time trong lúc chạy:
    #   ENTER       -> toggle obstacle front+rear
    #   g + ENTER   -> toggle ground

Output: <out_dir>/test_sim_<target>[_obs][_nognd]_simulated.xlsx
"""

import argparse
import sys
import threading

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser(description="Step response test in Isaac Sim")
parser.add_argument("--joint",    type=str,   default="right_knee_joint")
parser.add_argument("--targets",  type=float, nargs="+",
                    default=list(range(-45, 50, 5)))  # -45 → +45, bước 5°
parser.add_argument("--duration", type=float, default=3.0)
parser.add_argument("--out_dir",  type=str,   default="step_response_data")
parser.add_argument("--sign-override", type=float, default=None)
parser.add_argument("--fixed-base", action="store_true",
                    help="Use CHAOS_FIXED_CFG / fixed_base.usd for actuator identification.")

# --- Obstacle ---
obs_grp = parser.add_mutually_exclusive_group()
obs_grp.add_argument("--obstacle",    dest="obstacle", action="store_true",  default=False)
obs_grp.add_argument("--no-obstacle", dest="obstacle", action="store_false")
parser.add_argument("--obstacle-front", type=float, nargs=3, default=[0.15,  0.0, 0.05],
                    metavar=("X","Y","Z"))
parser.add_argument("--obstacle-rear",  type=float, nargs=3, default=[-0.10, 0.0, 0.05],
                    metavar=("X","Y","Z"))
parser.add_argument("--obstacle-size",  type=float, nargs=3, default=[0.20, 0.12, 0.06],
                    metavar=("SX","SY","SZ"))

# --- Ground ---
gnd_grp = parser.add_mutually_exclusive_group()
gnd_grp.add_argument("--ground",    dest="ground", action="store_true",  default=True,
                     help="Bật ground plane (default)")
gnd_grp.add_argument("--no-ground", dest="ground", action="store_false",
                     help="Tắt ground plane (teleport xuống -1000m)")

AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()
app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

# ── imports sau AppLauncher ──────────────────────────────────────────────────
import math
import os
import torch
import openpyxl
import omni.usd
from pxr import UsdGeom, Gf, UsdPhysics, PhysxSchema, Usd

import isaaclab.sim as sim_utils
from isaaclab.sim import SimulationContext
from isaaclab.assets import Articulation

from robolab.assets.robots.chaos import CHAOS_CFG, CHAOS_FIXED_CFG

# ── Constants ────────────────────────────────────────────────────────────────
PHYSICS_DT = 0.005
CONTROL_DT = 0.020
DECIMATION = int(CONTROL_DT / PHYSICS_DT)
DEG2RAD    = math.pi / 180.0
RAD2DEG    = 180.0 / math.pi
RADS2RPM   = 60.0 / (2.0 * math.pi)

GROUND_PRIM_PATH    = "/World/ground"
OBSTACLE_FRONT_PATH = "/World/obstacle_front"
OBSTACLE_REAR_PATH  = "/World/obstacle_rear"
OBSTACLE_PATHS      = [OBSTACLE_FRONT_PATH, OBSTACLE_REAR_PATH]


# ── Ground ───────────────────────────────────────────────────────────────────

def set_ground_active(stage, active: bool):
    """Teleport ground lên z=0 (active) hoặc xuống z=-1000 (inactive).
    Teleport đảm bảo PhysX thực sự không có collision, không dùng attribute toggle.
    """
    prim = stage.GetPrimAtPath(GROUND_PRIM_PATH)
    if not prim.IsValid():
        print(f"[Ground] Prim not found at {GROUND_PRIM_PATH}")
        return

    z = 0.0 if active else -1000.0
    xformable = UsdGeom.Xformable(prim)
    ops = xformable.GetOrderedXformOps()

    # Tìm translate op nếu có, không thì tạo mới
    translate_op = None
    for op in ops:
        if op.GetOpType() == UsdGeom.XformOp.TypeTranslate:
            translate_op = op
            break
    if translate_op is None:
        translate_op = xformable.AddTranslateOp()

    translate_op.Set(Gf.Vec3d(0.0, 0.0, z))
    status = "ON (z=0)" if active else "OFF (z=-1000)"
    print(f"[Ground] {status}")


# ── Obstacle ─────────────────────────────────────────────────────────────────

def create_obstacle_box(stage, prim_path, position, size, mass_kg=0.5):
    prim = stage.DefinePrim(prim_path, "Cube")

    xform = UsdGeom.Xformable(prim)
    xform.ClearXformOpOrder()
    xform.AddTranslateOp().Set(Gf.Vec3d(*position))
    xform.AddScaleOp().Set(Gf.Vec3d(size[0] / 2.0, size[1] / 2.0, size[2] / 2.0))

    UsdPhysics.RigidBodyAPI.Apply(prim)
    UsdPhysics.CollisionAPI.Apply(prim)
    mass_api = UsdPhysics.MassAPI.Apply(prim)
    mass_api.CreateMassAttr().Set(mass_kg)
    PhysxSchema.PhysxRigidBodyAPI.Apply(prim)

    print(f"[Obstacle] Created '{prim_path}' at {position}, size {size}")
    return prim


def _teleport_prim(stage, prim_path, active: bool, base_pos):
    """Teleport prim về base_pos (active) hoặc xuống -1000 (inactive)."""
    prim = stage.GetPrimAtPath(prim_path)
    if not prim.IsValid():
        return
    z = base_pos[2] if active else -1000.0
    xformable = UsdGeom.Xformable(prim)
    ops = xformable.GetOrderedXformOps()
    translate_op = None
    for op in ops:
        if op.GetOpType() == UsdGeom.XformOp.TypeTranslate:
            translate_op = op
            break
    if translate_op is None:
        translate_op = xformable.AddTranslateOp()
    translate_op.Set(Gf.Vec3d(base_pos[0], base_pos[1], z))


def set_obstacles_active(stage, active: bool, front_pos, rear_pos):
    """Teleport cả 2 box về vị trí gốc (active) hoặc xuống -1000 (inactive)."""
    _teleport_prim(stage, OBSTACLE_FRONT_PATH, active, front_pos)
    _teleport_prim(stage, OBSTACLE_REAR_PATH,  active, rear_pos)
    status = "ON" if active else "OFF"
    print(f"[Obstacle] Both obstacles {status}")


# ── Toggle thread ─────────────────────────────────────────────────────────────

class Toggler:
    """Lắng nghe input real-time:
        ENTER       -> toggle obstacle front+rear
        g + ENTER   -> toggle ground
    """

    def __init__(self, stage, obs_active: bool, gnd_active: bool,
                 front_pos, rear_pos):
        self.stage       = stage
        self.obs_active  = obs_active
        self.gnd_active  = gnd_active
        self.front_pos   = front_pos
        self.rear_pos    = rear_pos
        self._lock       = threading.Lock()
        self._thread     = threading.Thread(target=self._listen, daemon=True)
        self._thread.start()
        print("\n[Toggle] ENTER = obstacle ON/OFF  |  g + ENTER = ground ON/OFF\n")

    def _listen(self):
        while True:
            try:
                cmd = input().strip().lower()
                if cmd == "g":
                    self._toggle_ground()
                else:
                    self._toggle_obstacle()
            except EOFError:
                break

    def _toggle_obstacle(self):
        with self._lock:
            self.obs_active = not self.obs_active
            set_obstacles_active(self.stage, self.obs_active,
                                 self.front_pos, self.rear_pos)

    def _toggle_ground(self):
        with self._lock:
            self.gnd_active = not self.gnd_active
            set_ground_active(self.stage, self.gnd_active)

    def is_obs_active(self):
        with self._lock:
            return self.obs_active

    def is_gnd_active(self):
        with self._lock:
            return self.gnd_active


# ── Utilities ─────────────────────────────────────────────────────────────────

def shutdown(exit_code=0):
    if os.environ.get("CHAOS_FAST_EXIT", "0") == "1":
        sys.stdout.flush()
        sys.stderr.flush()
        os._exit(exit_code)
    try:
        simulation_app.close()
    finally:
        sys.stdout.flush()
        sys.stderr.flush()
        os._exit(exit_code)


def save_xlsx(out_path, target_deg, log):
    wb = openpyxl.Workbook()
    ws = wb.active
    ws.title = "Test Sim Data"
    ws.append(["Index (số sample):", len(log)])
    ws.append(["Target Position:", target_deg])
    ws.append([])
    ws.append(["Sample", "Time Step", "Actual Angle", "Actual RPM",
               "Torque (Nm)", "Obstacle", "Ground"])
    for i, e in enumerate(log):
        ws.append([
            i,
            int(e["t"]),
            round(e["pos_deg"],   4),
            round(e["vel_rpm"],   4),
            round(e["torque_nm"], 4),
            "ON" if e["obstacle"] else "OFF",
            "ON" if e["ground"]   else "OFF",
        ])
    wb.save(out_path)


def _safe(name, t, idx):
    try:
        return round(t[0, idx].item(), 5)
    except Exception:
        return f"<no {name}>"


# ── Core test loop ────────────────────────────────────────────────────────────

def run_one_test(robot, sim, joint_idx, n_joints, target_rad, duration,
                 sign=1.0, debug=False, toggler=None):
    zeros_pos = torch.zeros(1, n_joints)
    zeros_vel = torch.zeros(1, n_joints)
    robot.write_joint_state_to_sim(zeros_pos, zeros_vel)

    # Warm-up
    for _ in range(20):
        robot.set_joint_position_target(zeros_pos)
        robot.write_data_to_sim()
        sim.step()
        robot.update(PHYSICS_DT)

    target_pos = zeros_pos.clone()
    target_pos[0, joint_idx] = sign * target_rad

    n_steps = int(duration / CONTROL_DT)
    log     = []

    for step in range(n_steps):
        robot.set_joint_position_target(target_pos)
        robot.write_data_to_sim()
        for _ in range(DECIMATION):
            sim.step()
        robot.update(CONTROL_DT)

        obs_on = toggler.is_obs_active() if toggler else False
        gnd_on = toggler.is_gnd_active() if toggler else True

        if debug and step == 0:
            d = robot.data
            print(f"\n=== DEBUG step 0, joint idx={joint_idx} ===")
            print(f"  target_rad           : {target_rad:.5f}")
            print(f"  joint_pos_target     : {_safe('joint_pos_target', getattr(d,'joint_pos_target',None), joint_idx)}")
            print(f"  joint_pos            : {_safe('joint_pos', d.joint_pos, joint_idx)}")
            print(f"  joint_vel            : {_safe('joint_vel', d.joint_vel, joint_idx)}")
            print(f"  computed_torque      : {_safe('computed_torque', getattr(d,'computed_torque',None), joint_idx)}")
            print(f"  applied_torque       : {_safe('applied_torque', getattr(d,'applied_torque',None), joint_idx)}")
            print(f"  joint_stiffness (Kp) : {_safe('joint_stiffness', getattr(d,'joint_stiffness',None), joint_idx)}")
            print(f"  joint_damping   (Kd) : {_safe('joint_damping', getattr(d,'joint_damping',None), joint_idx)}")
            print(f"  joint_effort_limit   : {_safe('joint_effort_limit', getattr(d,'joint_effort_limit',None), joint_idx)}")
            lim = getattr(d, 'joint_pos_limits', None)
            if lim is None:
                lim = getattr(d, 'soft_joint_pos_limits', None)
            try:
                print(f"  joint_pos_limits     : [{lim[0,joint_idx,0].item():.4f}, {lim[0,joint_idx,1].item():.4f}]")
            except Exception:
                print(f"  joint_pos_limits     : <no limits field>")
            print(f"  obstacle             : {'ON' if obs_on else 'OFF'}")
            print(f"  ground               : {'ON' if gnd_on else 'OFF'}")
            print("=== END DEBUG ===\n")

        sample = {
            "t":         round(step * CONTROL_DT * 1000.0, 1),
            "pos_deg":   round(sign * robot.data.joint_pos[0, joint_idx].item() * RAD2DEG, 6),
            "vel_rpm":   round(sign * robot.data.joint_vel[0, joint_idx].item() * RADS2RPM, 6),
            "torque_nm": round(robot.data.applied_torque[0, joint_idx].item(), 6),
            "obstacle":  obs_on,
            "ground":    gnd_on,
        }
        log.append(sample)
        print(
            f"{sample['t']:>8.1f} | {sample['pos_deg']:>15.6f} | "
            f"{sample['vel_rpm']:>14.6f} | {sample['torque_nm']:>13.6f} | "
            f"obs={'ON ' if obs_on else 'OFF'} | gnd={'ON ' if gnd_on else 'OFF'}",
            flush=True,
        )

    if abs(log[-1]["pos_deg"]) < 0.1 and abs(target_rad * RAD2DEG) > 1.0:
        print(f"  [WARN] joint không di chuyển! Check joint_idx={joint_idx} và actuator config")

    return log


# ── Main ──────────────────────────────────────────────────────────────────────

def main():
    joint_out_dir = os.path.join(args_cli.out_dir, args_cli.joint)
    os.makedirs(joint_out_dir, exist_ok=True)

    sim = SimulationContext(sim_utils.SimulationCfg(dt=PHYSICS_DT, device="cpu"))
    sim.set_camera_view(eye=[1.5, 1.5, 0.8], target=[0.0, 0.0, 0.0])

    # Ground plane
    cfg_ground = sim_utils.GroundPlaneCfg(
        physics_material=sim_utils.RigidBodyMaterialCfg(
            static_friction=1.0,
            dynamic_friction=1.0,
            restitution=0.0,
        )
    )
    cfg_ground.func(GROUND_PRIM_PATH, cfg_ground)

    # Robot
    robot_cfg = CHAOS_FIXED_CFG if args_cli.fixed_base else CHAOS_CFG
    cfg = robot_cfg.replace(prim_path="/World/robot")
    cfg.spawn.func("/World/robot", cfg.spawn)
    robot = Articulation(cfg.replace(spawn=None))

    # Obstacle boxes (phải trước sim.reset())
    stage     = omni.usd.get_context().get_stage()
    obs_size  = tuple(args_cli.obstacle_size)
    front_pos = tuple(args_cli.obstacle_front)
    rear_pos  = tuple(args_cli.obstacle_rear)
    create_obstacle_box(stage, OBSTACLE_FRONT_PATH, front_pos, obs_size)
    create_obstacle_box(stage, OBSTACLE_REAR_PATH,  rear_pos,  obs_size)

    sim.reset()
    robot.reset()

    # Apply initial states từ CLI flags
    set_ground_active(stage, args_cli.ground)
    set_obstacles_active(stage, args_cli.obstacle, front_pos, rear_pos)

    # Toggler real-time
    toggler = Toggler(
        stage,
        obs_active=args_cli.obstacle,
        gnd_active=args_cli.ground,
        front_pos=front_pos,
        rear_pos=rear_pos,
    )

    # Tìm joint
    joint_ids, _ = robot.find_joints(args_cli.joint)
    if len(joint_ids) == 0:
        print(f"\nERROR: Không tìm thấy joint '{args_cli.joint}'")
        print(f"Joints có sẵn: {robot.joint_names}")
        shutdown(1)

    joint_idx = joint_ids[0]
    n_joints  = robot.num_joints

    # Lấy giới hạn khớp để skip các góc ngoài range
    limits  = robot.data.joint_pos_limits
    lo      = limits[0, joint_idx, 0].item()
    hi      = limits[0, joint_idx, 1].item()
    sign    = args_cli.sign_override if args_cli.sign_override is not None else 1.0

    print(f"\nAll joints : {list(enumerate(robot.joint_names))}")
    print(f"Joint      : {args_cli.joint} (idx={joint_idx})")
    print(f"Limits     : [{lo:.4f}, {hi:.4f}] rad  sign={sign:+.0f}")
    print(f"Steps      : {int(args_cli.duration/CONTROL_DT)} x {int(CONTROL_DT*1000)}ms = {args_cli.duration}s")
    print(f"Obstacle   : {'ON' if args_cli.obstacle else 'OFF'}  front={front_pos}  rear={rear_pos}  size={obs_size}")
    print(f"Ground     : {'ON' if args_cli.ground else 'OFF'}")
    print(f"Output     : {args_cli.out_dir}/\n")
    print(f"{'Target':>8} | {'Final(deg)':>10} | {'SS Error':>9} | "
          f"{'MaxRPM':>8} | {'Max rad/s':>10} | {'MaxTorque(Nm)':>13}")
    print("-" * 75)

    for i, target_deg in enumerate(args_cli.targets):
        target_rad = target_deg * DEG2RAD
        command_rad = sign * target_rad

        # Skip command angles outside joint limits, accounting for sign_override.
        if command_rad < lo or command_rad > hi:
            print(f"  [SKIP] {target_deg:+g} deg command={command_rad:+.4f} rad outside limits [{lo:.4f}, {hi:.4f}] rad")
            continue

        print(f"\n=== Target {target_deg:+g} deg  ({CONTROL_DT*1000:.0f}ms/sample) ===")
        print(f"{'t_ms':>8} | {'pos(deg)':>15} | {'rpm':>14} | "
              f"{'torque(Nm)':>13} | {'obs':>5} | {'gnd':>5}")
        print("-" * 72)

        log = run_one_test(
            robot, sim, joint_idx, n_joints, target_rad,
            args_cli.duration, sign=sign,
            debug=(i == 0), toggler=toggler,
        )

        final      = log[-1]["pos_deg"]
        ss_err     = target_deg - final
        max_rpm    = max(abs(e["vel_rpm"])   for e in log)
        max_vel    = max_rpm / RADS2RPM
        max_torque = max(abs(e["torque_nm"]) for e in log)

        suffix = ""
        if any(e["obstacle"] for e in log): suffix += "_obs"
        if any(not e["ground"] for e in log): suffix += "_nognd"
        out_path = os.path.join(joint_out_dir,
                                f"test_sim_{int(target_deg)}{suffix}_simulated.xlsx")
        save_xlsx(out_path, target_deg, log)

        print(f"{target_deg:>7}d | {final:>10.3f} | {ss_err:>9.3f} | "
              f"{max_rpm:>8.1f} | {max_vel:>10.3f} | {max_torque:>13.3f}")

    print(f"\nDone. Files: {os.path.abspath(joint_out_dir)}/")
    shutdown(0)


if __name__ == "__main__":
    main()
