<div align="center">
  <img src="https://raw.githubusercontent.com/Ngoc411/Humanoid-lower-body-12dof/main/flat_ter.gif" width="600">
  <br>
  <em>Chaos — a 12-DoF humanoid lower-body robot, sim-to-real RL locomotion</em>
</div>

# Humanoid-lower-body-12dof (Chaos)

This repository holds the full stack for **Chaos**, a 12-degree-of-freedom humanoid
lower-body robot (6 DoF per leg: hip yaw / hip roll / hip pitch / knee / ankle pitch /
ankle roll): reinforcement-learning training in simulation, the ONNX policy runtime,
and the embedded firmware that runs on the physical robot.

> **Acknowledgement / reference:** the simulation and RL training pipeline in this
> repo is built on top of, and takes reference from,
> **[VinRobotics/vinrobotics_mjlab](https://github.com/VinRobotics/vinrobotics_mjlab)**.
> Credit to that project and its authors for the base framework this work extends.

## Project layout

```
.
├── source/Vin_Rsl_Rl/          # Isaac Lab-style RL extension (Python package "Vin_Rsl_Rl")
│   └── Vin_Rsl_Rl/
│       ├── assets/robots/      # Robot articulation config ("chaos.py": joints, actuators, PD gains)
│       └── tasks/              # Gym-style task definitions (manager-based & direct)
│           ├── manager_based/vin_rsl_rl/   # MDP: rewards, terminations, curriculum, observations
│           └── direct/                     # Direct-workflow + multi-agent (MARL) variants
├── scripts/                    # Train / play entry points for each RL library
│   ├── rsl_rl/                 # train.py, play.py, plot_policy_data.py (primary pipeline)
│   ├── rl_games/                # train.py, play.py
│   ├── sb3/                    # train.py, play.py
│   └── skrl/                    # train.py, play.py
├── step_response_data/         # Real-motor step-response / wave-motion logs used to fit
│                                # simulated actuator dynamics (armature, friction, viscous
│                                # friction) per joint against the physical robot
├── ws1_cpp/ws1_cpp/             # ROS 2 C++ workspace that runs on the Jetson (deploys the
│   ├── sensor_node/             #   trained ONNX policy on hardware)
│   ├── src/                     # sensor_node : SPI master, reads 46-float obs frame from STM32
│   ├── action_node/             # main_node   : runs actor.onnx, fall/recovery state machine
│   └── test_model_node/         # action_node : writes the 12-joint action frame back to STM32
│                                 # test_model_node : offline CLI sanity check of the ONNX model
├── Chaos_main_1/                # STM32F407VET6 firmware — main board / SPI slave to the
│                                 # Jetson, CAN master to the 12 per-joint motor nodes
│   ├── NODE_FLOW.md             # Full node/data-flow reference for the STM↔Jetson link
│   ├── JETSON_NOTES.txt         # Fall-detection & recovery handshake, data-safety checklist
│   └── STM32_SAFETY_SUMMARY.txt # NaN/CRC/framing/watchdog protections on the STM32 side
├── Chaos_test3/                  # STM32G474 firmware flashed to each of the 12 joint nodes
│                                 # (one CAN node per motor; NODE_ID selects which joint)
├── Hardware/                      # Mechanical CAD (SolidWorks) for the physical robot
│   ├── hip_2/hip_3/hip_connector, knee_1/2_left/right, ankle*  # Structural leg parts
│   ├── gearbox/                   # Custom planetary gearboxes for the BLDC actuators
│   │   ├── gearbox_5210/          #   (5210-size motor)
│   │   └── gearbox_3508/, gearbox_3508_2/  # (3508-size motor, two revisions)
│   ├── BLDC/                      # Motor mount parts for the 5210 / 3508 / 4108 BLDC motors
│   ├── bearing/                   # Bearing mounts
│   ├── assem_symplified.SLDASM    # Full simplified leg/body assembly
│   └── Jetson Nano.step           # Reference model for mounting the onboard computer
├── test7.usd                      # Isaac Sim USD asset for Chaos, referenced as
│                                  # `usd_path` in `source/.../assets/robots/chaos.py`
└── flat_ter.gif / flat_ter.mov   # Walking demo captured from simulation
```

## System overview

Chaos is trained to walk in simulation and deployed on hardware through a three-tier
control stack:

```
STM32F407 (main board)  ──CAN──►  12× STM32G474 joint nodes (motor control)
        ▲  │
       SPI │  46-float frames, 50 Hz
        │  ▼
  Jetson Orin Nano  ──►  ONNX policy (actor.onnx)  ──►  action back to STM32
```

1. **Training (simulation).** `source/Vin_Rsl_Rl` defines the Chaos robot (joint
   limits, PD gains, actuator dynamics) and locomotion task as an Isaac
   Lab-style extension. Actuator parameters (armature, friction, viscous
   friction per joint) were fit against real step-response and wave-motion
   data recorded from the physical motors (`step_response_data/`) so the
   simulated dynamics track the real robot.
2. **Policy export.** A trained policy is exported to `actor.onnx` for
   deployment.
3. **On-robot inference (Jetson).** The `ws1_cpp` ROS 2 workspace runs three
   nodes: `sensor_node` (SPI master, pulls the 46-float observation frame from
   the STM32 and converts deg/RPM → rad/rad·s⁻¹), `main_node` (runs the ONNX
   policy, owns the INIT → RUNNING → FALLEN state machine and fall/recovery
   logic, applies ankle linkage math and safety clamps), and `action_node`
   (converts the 12 joint commands back to degrees and writes them to the
   STM32). See `Chaos_main_1/NODE_FLOW.md` for the full frame layout and
   state-machine reference.
4. **Embedded control (STM32).** `Chaos_main_1` is the main board: it bridges
   the Jetson (SPI slave) and the 12 joint nodes (CAN master), and is
   responsible for the WAIT/READY handshake, fault handling, and deciding
   whether an action is safe to forward. `Chaos_test3` is the firmware flashed
   to each of the 12 individual joint/motor nodes, one per joint, selected by
   `NODE_ID` at flash time.

<div align="center">
  <img src="https://raw.githubusercontent.com/Ngoc411/Humanoid-lower-body-12dof/main/setup.jpg" width="500">
  <br>
  <em>Physical test rig: Chaos suspended from a tether frame while the RL
  policy runs live on the Jetson (foreground), so early policies can be
  tested on hardware without risking a fall.</em>
</div>

## Getting started

### Prerequisites

The `source/Vin_Rsl_Rl` package is an [Isaac Lab](https://isaac-sim.github.io/IsaacLab/)
extension, which itself runs on top of
[NVIDIA Isaac Sim](https://developer.nvidia.com/isaac/sim). Install both before
using this repo:

- **Isaac Sim** — see the
  [Isaac Sim installation guide](https://docs.isaacsim.omniverse.nvidia.com/latest/installation/index.html).
  This project was developed against Isaac Sim 4.5 / 5.0 / 5.1.
- **Isaac Lab** — see the
  [Isaac Lab installation guide](https://isaac-sim.github.io/IsaacLab/main/source/setup/installation/index.html)
  ([GitHub](https://github.com/isaac-sim/IsaacLab)). `source/Vin_Rsl_Rl` follows
  the standard Isaac Lab extension template, so it installs the same way as
  any other external Isaac Lab task extension.

### Simulation / RL training

```bash
# from the repo root, with an Isaac Lab environment set up
python -m pip install -e source/Vin_Rsl_Rl

# train with RSL-RL (the primary pipeline used for this robot)
python scripts/rsl_rl/train.py --task <task-name>

# play back a trained checkpoint
python scripts/rsl_rl/play.py --task <task-name> --checkpoint <path-to-checkpoint>
```

A trained checkpoint is already included under `logs/rsl_rl/`, so you can skip
training and watch the policy walk right away:

```bash
python scripts/rsl_rl/play.py \
  --task Template-Vin-Rsl-Rl-Flat-Play-v0 \
  --checkpoint logs/rsl_rl/2026-07-04_21-39-15/model_14000.pt
```

The exported ONNX/TorchScript versions of that same policy (used on the real
robot) are already available at
`logs/rsl_rl/2026-07-04_21-39-15/exported/{policy.onnx,policy.pt}` — no export
step needed.

> **Robot USD asset:** `chaos.py` currently points `usd_path` at a local
> development path (`D:/IsaacSim/test7.usd`). The actual asset is checked into
> this repo at `test7.usd`; update `usd_path` in
> `source/Vin_Rsl_Rl/Vin_Rsl_Rl/assets/robots/chaos.py` to point at your local
> copy of that file (or move/symlink it into your Isaac Sim assets folder)
> before running training or play.

Equivalent `train.py` / `play.py` entry points are also provided for
`rl_games`, `sb3`, and `skrl` under `scripts/`.

### Hardware deployment

The `ws1_cpp` workspace is a ROS 2 package intended to run on the Jetson Orin
Nano that is connected to the STM32F407 main board over SPI:

```bash
cd ws1_cpp/ws1_cpp
colcon build
source install/setup.bash
ros2 launch <package> <launch-file>   # or run sensor_node / main_node / action_node individually
```

`main_node` loads `actor.onnx` via ONNX Runtime; place the exported policy
where the node expects it before launching. `test_model_node` can be used to
sanity-check a model offline without any ROS 2 topics or SPI hardware:

```bash
ros2 run ws1_cpp test_model_node <45 space-separated observation floats>
```

### Firmware

`Chaos_main_1` and `Chaos_test3` are STM32CubeIDE projects (`.ioc` files
included). Open each in STM32CubeIDE to build and flash:
- **`Chaos_main_1`** → STM32F407VET6, the main/SPI-slave board. Bridges the
  Jetson (SPI3, DMA, 46-float / 184-byte frames each direction, triggered at
  50 Hz by the Jetson) and the 12 joint nodes over CAN. It polls all 12 nodes
  for feedback, forwards actions only when the system is armed and safe, and
  owns the fall-detection broadcast (CAN ID `150`) that puts every joint node
  into a homing state.
- **`Chaos_test3`** → STM32G474RETx, flashed once per joint node with a
  unique `NODE_ID` (1–12); all of that node's CAN IDs (command, feedback,
  init-done) are derived from `NODE_ID`. Each node runs its own motor control
  loop and reports position/velocity feedback back to the main board.

Both boards share a protection library (`lib/Protection` in `Chaos_main_1`)
covering NaN/Inf checks, clamping, CRC32 framing, sequence checks, a
watchdog, and slew-rate limiting — see `STM32_SAFETY_SUMMARY.txt` for the
full breakdown of what's implemented on the STM32 side versus what's still
planned (e.g. CRC/sequence numbers are not yet enabled on the SPI link).

### Hardware (mechanical design)

`Hardware/` contains the SolidWorks source files for the physical robot: the
structural leg parts (hip, knee, ankle links and connectors), the custom
planetary gearboxes built for the 5210 / 3508 BLDC actuators, the BLDC motor
mounts, bearing mounts, a simplified full-assembly file
(`assem_symplified.SLDASM`), and a reference STEP model for mounting the
Jetson Nano. Open the `.SLDPRT` / `.SLDASM` files in SolidWorks (or import the
`.STEP` files into other CAD tools) to inspect or modify the design.

## Safety design

The STM32 ↔ Jetson link and the on-robot policy runtime include several
fail-safe layers, documented in detail in `Chaos_main_1/NODE_FLOW.md`,
`Chaos_main_1/JETSON_NOTES.txt`, and `Chaos_main_1/STM32_SAFETY_SUMMARY.txt`:

- **Fall detection & recovery handshake** — both the STM32 and the Jetson
  independently estimate tilt from IMU data; recovery requires both sides to
  agree the robot has fallen and later stood back up before motion resumes.
- **NaN/Inf handling** — observations are never clamped (to avoid feeding the
  policy false data), only checked for finite values; actions are checked for
  finite values *before* clamping, since clamping propagates NaNs.
- **Joint limit clamping** and a startup/init handshake that requires all 12
  joint nodes to report ready before any action is forwarded.
- **Link-loss watchdog** — a stale observation stream (e.g. SPI desync, cable
  fault) forces the robot into a safe "fallen/hold" state rather than acting on
  outdated data.

## Notes

- `chaos.py` still has a hardcoded `D:/IsaacSim/...` path for the fixed-base
  variant (`fixed_base.usd`, used only for actuator step-response
  identification); that file isn't included in this repo, so update or remove
  that reference if you don't need the fixed-base bench.
- Some in-code comments and internal docs are written in Vietnamese; the
  English summaries above and in this README cover the key behavior.

---

## Author

**Ngoc**

Embedded Systems · Motor Control · Robotics
