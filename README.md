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

## Getting started

### Simulation / RL training

The `source/Vin_Rsl_Rl` package is an Isaac Lab extension and expects an
Isaac Lab / Isaac Sim installation.

```bash
# from the repo root, with an Isaac Lab environment set up
python -m pip install -e source/Vin_Rsl_Rl

# train with RSL-RL (the primary pipeline used for this robot)
python scripts/rsl_rl/train.py --task <task-name>

# play back a trained checkpoint
python scripts/rsl_rl/play.py --task <task-name> --checkpoint <path-to-checkpoint>
```

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
- `Chaos_main_1` → STM32F407VET6 main/SPI-slave board.
- `Chaos_test3` → STM32G474RETx, flashed once per joint node with a unique
  `NODE_ID` (1–12).

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

- Simulation asset paths (`D:/IsaacSim/...`) in `chaos.py` are local to the
  development machine used for training and will need to be updated to your
  own USD asset locations.
- Some in-code comments and internal docs are written in Vietnamese; the
  English summaries above and in this README cover the key behavior.
