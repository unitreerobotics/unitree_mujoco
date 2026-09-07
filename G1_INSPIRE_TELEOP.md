# G1 + Inspire Hand MuJoCo Teleop Bridge

This document summarizes the work done to enable **Quest 3 → xr_teleoperate → MuJoCo** teleoperation for a **G1 29-DoF upper body with Inspire DFX hands**, as a Sim2Sim alternative to Isaac Lab.

---

## Goal

Replace the Isaac Sim backend with MuJoCo while keeping **xr_teleoperate unchanged**. Both sides communicate over the same Unitree DDS topics used on the real robot and in Isaac simulation.

```
Quest 3 → xr_teleoperate → DDS → MuJoCo (g1_inspire_sim)
```

Scope: **upper body only** (arms + waist + Inspire hands). Pelvis is fixed to the world so the robot does not fall without a walking policy.

---

## What Was Built

### Phase 0 — MuJoCo model (G1 + Inspire, pelvis fixed)

- Converted Inspire hand URDFs from `xr_teleoperate/assets/inspire_hand/` into MuJoCo body snippets.
- Merged them onto `g1_29dof.xml` at both wrist yaw links using the official mount transform from `h1_2.urdf`:
  - Left:  `pos="0.054 0 0"`, `quat="0.707 0 0 0.707"`
  - Right: `pos="0.054 0 0"`, `quat="0 0.707 -0.707 0"`
- Removed the pelvis free joint so the robot stands still.
- Added 12 hand position actuators in **Inspire DDS motor order** (ids 0–5 right, 6–11 left).
- Added equality constraints for URDF mimic joints (intermediate/distal fingers follow proximal joints).
- Replaced default rubber-hand geoms with Inspire hand meshes.

**Output files:**

| File | Description |
|------|-------------|
| `unitree_robots/g1/g1_29dof_inspire_fixed.xml` | Full robot model (41 actuators: 29 body + 12 hand) |
| `unitree_robots/g1/scene_29dof_inspire_fixed.xml` | Scene wrapper (floor, lighting, skybox) |
| `unitree_robots/g1/inspire_build/convert_hands.py` | URDF → MJCF conversion script |
| `unitree_robots/g1/inspire_build/merge_g1_inspire.py` | Model merge script (re-runnable) |

### Phase 1 — Body DDS bridge (arms + waist + legs hold)

- Subscribes to `rt/lowcmd` (`unitree_hg` LowCmd_, 29 motors).
- Applies PD control: `tau = tau_ff + kp*(q_target - q) + kd*(dq_target - dq)`.
- Publishes `rt/lowstate` with joint positions, velocities, torques, and IMU data.
- Joints not yet commanded are held at their current pose with default hold gains (`kp=60`, `kd=1.5`) so legs do not swing freely.

### Phase 2 — Inspire hand DDS bridge

- Subscribes to `rt/inspire/cmd` (`unitree_go` MotorCmds_, 12 motors, normalized 0–1).
- Publishes `rt/inspire/state` continuously (**required** — `Inspire_Controller_DFX` in xr_teleoperate blocks until it receives hand state).
- Maps normalized commands to joint angles using the same ranges as Isaac `inspire_dds.py` and xr_teleoperate.

**Inspire DDS motor order (12 channels):**

| ID | Joint | Range (rad) |
|----|-------|-------------|
| 0 | R_pinky_proximal | 0.0 – 1.7 |
| 1 | R_ring_proximal | 0.0 – 1.7 |
| 2 | R_middle_proximal | 0.0 – 1.7 |
| 3 | R_index_proximal | 0.0 – 1.7 |
| 4 | R_thumb_proximal_pitch | 0.0 – 0.5 |
| 5 | R_thumb_proximal_yaw | -0.1 – 1.3 |
| 6 | L_pinky_proximal | 0.0 – 1.7 |
| 7 | L_ring_proximal | 0.0 – 1.7 |
| 8 | L_middle_proximal | 0.0 – 1.7 |
| 9 | L_index_proximal | 0.0 – 1.7 |
| 10 | L_thumb_proximal_pitch | 0.0 – 0.5 |
| 11 | L_thumb_proximal_yaw | -0.1 – 1.3 |

Normalization: `q_norm = (max - q) / (max - min)` where **1.0 = fully open**, **0.0 = fully closed**.

### Phase 3 — Integration entry point

- `g1_inspire_sim.py` runs the MuJoCo viewer + physics loop + DDS bridge on **domain 1** (same as `xr_teleoperate --sim`).
- No changes to xr_teleoperate are required; only the simulator backend is swapped.

---

## New Files (simulate_python/)

| File | Role |
|------|------|
| `g1_inspire_bridge.py` | DDS ↔ MuJoCo bridge (body + Inspire hands) |
| `g1_inspire_sim.py` | Main sim launcher (viewer + physics + bridge) |
| `test/test_g1_inspire_teleop.py` | Standalone test script (fake teleop without Quest) |

---

## DDS Topics

| Topic | Direction | Message type | Purpose |
|-------|-----------|--------------|---------|
| `rt/lowcmd` | teleop → sim | `unitree_hg` LowCmd_ | Body joint targets (29 motors) |
| `rt/lowstate` | sim → teleop | `unitree_hg` LowState_ | Body joint feedback + IMU |
| `rt/inspire/cmd` | teleop → sim | `unitree_go` MotorCmds_ | Hand targets (12 motors, 0–1) |
| `rt/inspire/state` | sim → teleop | `unitree_go` MotorStates_ | Hand feedback (12 motors, 0–1) |

DDS domain: **1** for simulation (matches `ChannelFactoryInitialize(1)` in xr_teleoperate when `--sim` is set).

---

## How to Run

### Prerequisites

- Conda env `unitree` with `mujoco` and `unitree_sdk2py` installed.
- Conda env `tv` with xr_teleoperate dependencies installed.
- Quest 3 and host PC on the same Wi-Fi (for Phase 3).

### Terminal 1 — MuJoCo sim

```bash
conda activate unitree
cd ~/Documents/fibo/project_humanoid/unitree_mujoco/simulate_python
python g1_inspire_sim.py
```

Options:

- `--headless` — run without MuJoCo viewer (for automated testing).
- `--interface lo` — DDS network interface (default: `lo`; use your LAN interface if teleop runs on another machine).

### Terminal 2 — xr_teleoperate

```bash
conda activate tv
cd ~/Documents/fibo/project_humanoid/xr_teleoperate/teleop
python teleop_hand_and_arm.py --arm=G1_29 --ee=inspire_dfx --sim --body-tracking upper
```

### Quest 3

1. Open `https://<HOST_IP>:8012/?ws=wss://<HOST_IP>:8012` in the headset browser.
2. Click **Virtual Reality** and accept certificate prompts.
3. Align your arms to the robot's initial pose (arms at sides).
4. Press **`r`** in the teleop terminal to start tracking.

### Standalone test (no Quest)

With `g1_inspire_sim.py --headless` running:

```bash
conda activate unitree
cd ~/Documents/fibo/project_humanoid/unitree_mujoco/simulate_python
python test/test_g1_inspire_teleop.py
```

---

## Test Results (automated)

| Test | Result |
|------|--------|
| Model loads (`scene_29dof_inspire_fixed.xml`) | Pass — 41 actuators, 53 joints, 12 equality constraints |
| `rt/lowstate` published | Pass |
| `rt/inspire/state` published | Pass — initial values ~1.0 (hands open) |
| Arm tracking (shoulder/elbow/wrist/waist) | Pass — targets reached within ~5% at teleop-like gains |
| Hand open/close (cmd 1.0 / 0.0 / 0.5) | Pass — all 12 channels track normalized commands |
| Mimic joints (finger intermediate/distal) | Pass — coupled via equality constraints |

---

## Architecture Diagram

```
┌─────────────┐     WebXR/Vuer      ┌──────────────────┐
│   Quest 3   │ ◄──────────────────►│  xr_teleoperate  │
└─────────────┘                     │  (teleop_hand_   │
                                    │   and_arm.py)    │
                                    └────────┬─────────┘
                                             │ CycloneDDS (domain 1)
                              ┌──────────────┼──────────────┐
                              │              │              │
                         rt/lowcmd    rt/inspire/cmd   (no image
                              │              │           server in
                              ▼              ▼           MuJoCo MVP)
                    ┌─────────────────────────────┐
                    │      g1_inspire_sim.py      │
                    │  ┌───────────────────────┐  │
                    │  │  g1_inspire_bridge.py │  │
                    │  │  - LowCmdHandler      │  │
                    │  │  - InspireCmdHandler  │  │
                    │  │  - PublishLowState    │  │
                    │  │  - PublishInspireState│  │
                    │  └───────────┬───────────┘  │
                    │              │              │
                    │  ┌───────────▼───────────┐  │
                    │  │  MuJoCo physics step  │  │
                    │  │  g1_29dof_inspire_    │  │
                    │  │  fixed.xml (41 ctrl)  │  │
                    │  └───────────────────────┘  │
                    └─────────────────────────────┘
                              │              │
                         rt/lowstate   rt/inspire/state
                              │              │
                              └──────┬───────┘
                                     ▼
                              xr_teleoperate
                              (feedback / state)
```

---

## Known Limitations

1. **No camera feed** — Unlike Isaac Lab, this MuJoCo setup does not stream a robot POV image to Quest. Teleop still works for arm/hand control; immersive view requires a separate image pipeline (e.g. teleimager) if needed later.
2. **No `rt/sim_state`** — Recording with `--record` will have empty `sim_state` fields. Teleop and hand/arm control work without it.
3. **Fixed pelvis** — Robot cannot walk or fall. Leg joints are held in place. Suitable for upper-body teleop dev only.
4. **Hand mount transform** — Uses H1_2 URDF mount offsets. Fine-tune in `inspire_build/merge_g1_inspire.py` if finger orientation looks wrong in VR.
5. **Shoulder tracking under gravity** — At teleop gains (`kp=80`), shoulder pitch may not reach large forward targets fully due to arm weight in simulation. This matches expected sim2real behavior; increase gains or add feedforward if needed.

---

## Rebuilding the Model

If you change hand meshes, mount pose, or joint limits:

```bash
conda activate unitree
cd ~/Documents/fibo/project_humanoid/unitree_mujoco/unitree_robots/g1/inspire_build
python convert_hands.py    # URDF → MJCF snippets
python merge_g1_inspire.py # merge into g1_29dof_inspire_fixed.xml + scene
```

Inspire hand STL meshes are copied into `unitree_robots/g1/meshes/` from `xr_teleoperate/assets/inspire_hand/meshes/`.

---

## Relation to Other Repos

| Repo | Role in this stack |
|------|-------------------|
| `xr_teleoperate` | Quest 3 input, IK, hand retargeting, DDS command publisher — **unchanged** |
| `unitree_mujoco` | MuJoCo sim + DDS bridge — **this work** |
| `unitree_sim_isaaclab` | Original Isaac Sim backend (replaced for this dev path) |
| `deploy/` | Locomotion RL policy deploy — **separate concern**, not used for teleop |

---

## Next Steps (optional)

- [ ] Add MuJoCo offscreen camera → teleimager for Quest POV in sim
- [ ] Publish stub `rt/sim_state` for `--record` compatibility
- [ ] Sim2Real: drop `--sim`, set DDS domain 0 + robot network interface
- [ ] Tune hand mount transform after first Quest session
- [ ] Add `--no_g1_state_pub`-style DDS isolation if multicast issues appear (see `unitree_sim_isaaclab/CODEX_CONTEXT.md`)
