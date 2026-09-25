# ws1_cpp — Node Flow Reference

Robot: **STM32F407 (SPI slave) ↔ Jetson Orin Nano (SPI master + RL policy)**.
Per `JETSON_NOTES.txt`. Units on the wire are **deg / RPM**; rad / rad/s are
Jetson-internal only. SPI frame = **46 floats (184 bytes) each direction**.

```
            sensor_45_float (46 floats)            main/output (12 floats)
 STM ──SPI──►  sensor_node  ──────────────►  main_node  ──────────────►  action_node  ──SPI──► STM
 (obs+STATUS)                (obs converted)            (action, rad)                  (action, deg)
```

Three independent nodes, two separate SPI transactions on `/dev/spidev0.0`
(sensor reads obs; action writes commands).

---

## Frame layouts

### Obs frame — STM → Jetson (read by sensor_node), 46 floats
| idx | field | unit on wire | Jetson converts to |
|-----|-------|--------------|--------------------|
| 0–2   | base angular velocity | rad/s | (unchanged) |
| 3–5   | projected gravity (normalized) | – | (unchanged) |
| 6–8   | velocity commands | – | (unchanged) |
| 9–20  | joint position (12) | deg | rad |
| 21–32 | joint velocity (12) | RPM | rad/s |
| 33–44 | prev action (12) | deg | rad |
| **45** | **STATUS** | 0=WAIT, 1=READY | published raw |

### Action frame — Jetson → STM (written by action_node), 46 floats
| idx | field |
|-----|-------|
| 0     | 2.0  (CMD_ACTION marker) |
| 1–12  | 12 joint commands (deg) |
| 13–45 | padding (0.0) |

---

## 1. sensor_node  (`sensor_node/sensor_node.cpp`)

**Role:** SPI master. Triggers the STM at a fixed rate, reads the 46-float obs
frame, converts units, publishes it. Build-time `TEST` macro selects mode
(`0` = production, `1` = GPIO+SPI bench, `2` = SPI loopback). Production is `#else`.

**GPIO lines**
- Pin 15 (line 85): OUTPUT — trigger pulse.
- Pin 7  (line 144): INPUT — EXTI rising edge from STM ("data ready").

**Startup**
1. `openSPI()` — open `/dev/spidev0.0`, set mode 0, 8-bit, 1 MHz.
2. `openGPIO()` — request pin 15 output + pin 7 rising-edge events; launch
   `gpioWatchThread`.
3. Create publisher `sensor_45_float` and a **100 Hz wall timer** → `timerCallback`.

**Trigger loop — `timerCallback` (every 10 ms)**
- If no SPI transfer is in progress, **toggle pin 15** (H→L→H→L…), recording the
  timestamp only on the HIGH edge. Net result = 50 Hz square wave; each rising
  edge is the trigger the STM waits on.

**EXTI loop — `gpioWatchThread` (separate thread)**
1. Block on `gpiod_line_event_wait(pin7)`.
2. On rising edge: log dt between edges + latency from pin-15 HIGH.
3. Set `transfer_in_progress_`, force pin 15 LOW, sync `pulse_state_`.
4. `doSPITransfer()`, then clear `transfer_in_progress_`.

**`doSPITransfer`**
1. Full-duplex `SPI_IOC_MESSAGE` of `rx_len_` (46×4 = 184 B). TX is don't-care.
2. `memcpy` rx → 46 floats.
3. Convert in place: `[9-20]` deg→rad, `[21-32]` RPM→rad/s, `[33-44]` deg→rad.
   `[45]` STATUS left raw.
4. Publish all 46 floats on `sensor_45_float`.

---

## 2. main_node  (`src/main.cpp`, `src/main.hpp`)

**Role:** the brain. Subscribes obs, runs the ONNX policy, applies linkage math +
safety, and owns the **fall / READY state machine**. Publishes 12 action
floats (rad) on `main/output`. `TEST 0` here = a standalone linkage unit-test node;
`#else` (active) = `MainNode`.

**Startup (`MainNode()`)**
1. Init ONNX Runtime, load `actor.onnx` (single-thread).
2. Fetch I/O names; pre-allocate 45-in / 12-out buffers.
3. Subscribe `sensor_45_float`; publish `main/output`.
4. Start a **10 Hz watchdog** timer → `watchdogTick`. Initial state = `INIT`.

**Per-message — `sensorDataCallback`**
1. **Framing:** require exactly 46 floats. Split `obs = [0..44]`, `status = [45]`.
2. **Obs fail-safe:** any non-finite obs → `nan_obs_count_++`, drop frame, return.
3. Stamp `last_valid_obs_` (feeds watchdog).
4. **State machine** — 3 states, `INIT → RUNNING → FALLEN → INIT`. The 5 s warmup
   lives on the **STM32**, not the Jetson. STATUS is consulted **only in INIT**.

```
INIT      status==READY ──────────────────────────► RUNNING (reset fall debounce)
          (else: stay, no action out)

RUNNING   detectFall() true ──► resetPolicyState() ► FALLEN
          else ──────────────────────────────────► runInference()  (publishes)
          (STATUS is IGNORED here — only a fall leaves RUNNING)

FALLEN    detectStanding() true ──────────────────► INIT  (await READY again)
          (model off, prev_action/filter zeroed; only standing detection runs)
```

   - **Fall** (`detectFall`, RUNNING): `tilt = atan2(hypot(gx,gy), -gz) > 20°` for
     **3 consecutive** cycles.
   - **Standing** (`detectStanding`, FALLEN): same tilt back **< 20°** for 3
     consecutive cycles. Obs keeps flowing during a fall, so this works while the
     robot is being re-homed.
   - Recovery needs **both** signals, split across states: Jetson sees the robot
     stand up (`FALLEN→INIT`), then the STM declares `READY` (`INIT→RUNNING`).
   - Only `RUNNING` ever calls `runInference` / publishes. INIT & FALLEN are silent
     (STM holds the last safe pose).

**`runInference` (RUNNING only)**
1. CSV session logging: when `obs[6,7,8]` (commands) go non-zero, open a new
   `logs/session_NNN.csv`; on all-zero, close it. (Only active while RUNNING.)
2. Copy first 45 obs into the model buffer.
3. **Pre-model:** inverse-linkage the two ankle pairs (motor θ_D/θ_C → rot_x/rot_y)
   for position + velocity; overwrite `obs[33-44]` with last cycle's raw action;
   subtract pose offset.
4. **Run ONNX** → 12 raw outputs.
5. **Action fail-safe:** any non-finite output → `nan_action_count_++`, return
   (checked **before** clamp, since `clamp(NaN)=NaN`).
6. **Post-model:** `out·SCALE + poseOffset`, clip to per-joint limits
   (count clips), store raw as next-cycle prev_action, optional EMA low-pass.
7. **Forward linkage** on ankle pairs (rot_x/rot_y → θ_D/θ_C).
8. Publish 12 floats (rad) on `main/output`; append CSV row.

**`watchdogTick` (10 Hz, runs independently of obs messages)**
- Every valid obs stamps `last_valid_obs_`. The watchdog checks the gap since then.
- If RUNNING and no valid obs for **>200 ms** (obs stream died — cable, STM crash,
  SPI desync) → `stale_count_++`, `resetPolicyState()`, → FALLEN. When obs stops,
  the message callback also stops, so this timer is the only thing that can still
  react and keep the robot from acting on a dead link.
- Every ~5 s log `[STATS] nan_obs / nan_action / clip / stale / state`.

**`detectFall` / `detectStanding` / `resetPolicyState`**
- `detectFall` (RUNNING): debounced `tilt > 20°`. `detectStanding` (FALLEN):
  debounced `tilt < 20°`. ⚠️ Both assume `gz≈-1` upright — verify against logged
  obs[3-5]; flip `-gz` in both if your convention reads `+1`.
- `resetPolicyState`: zero prev_action + filter state + both debounce counters,
  close any open log.

---

## 3. action_node  (`action_node/action_node.cpp`, `.hpp`)

**Role:** SPI master for the command direction. Subscribes the policy output and
writes a 46-float action frame to the STM.

**Startup:** open `/dev/spidev0.0` (mode 0, 8-bit, 1 MHz); subscribe `main/output`.

**Per-message — `commandCallback`**
1. Require exactly 12 floats.
2. **Fail-safe:** any non-finite command → log + **do not send**.
3. Convert rad → deg.
4. Build 46-float frame: `[0]=2.0`, `[1-12]=cmd°`, `[13-45]=0`.
5. `sendSPI` — TX-only `SPI_IOC_MESSAGE` (184 B).

---

## 4. test_model_node  (`test_model_node/test_model_node.cpp`)

**Role:** offline CLI sanity check of the ONNX model — no ROS spin, no SPI.

**Flow:** parse 45 floats from argv → load `actor.onnx` → single inference →
print the 12 raw outputs space-separated → exit.

```bash
ros2 run ws1_cpp test_model_node 0.0 0.0 ... (45 numbers)
```

---

## State / safety summary

| Concern | Where | Behavior |
|---------|-------|----------|
| WAIT/READY handshake | main_node INIT only | RUNNING starts on STM READY; 5 s warmup is on the STM |
| Fall detection | main_node `detectFall` (RUNNING) | tilt >20°, debounce 3 → FALLEN |
| Recovery | main_node FALLEN→INIT→RUNNING | Jetson sees standing (→INIT), then STM READY (→RUNNING) |
| Obs corruption | main_node | non-finite → drop frame |
| Action corruption | main_node + action_node | non-finite → don't publish/send |
| Joint limits | main_node | clip + count |
| Link loss | main_node watchdog | stale >200 ms → FALLEN |
| Unit conversion | sensor (in) / action (out) | deg↔rad, RPM→rad/s |

**Not yet implemented** (needs matching STM work, per spec): CRC32, sequence
numbers, magic-header framing.
```
