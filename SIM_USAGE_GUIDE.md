# Simulator Usage Guide

How to run MuJoCo simulation in this repo, configure parameters, and switch between **sim → sim** and **sim → real**.

Controllers talk the same Unitree DDS topics as the physical robot. Switching backends is mostly **domain ID + network interface**.

```
Controller (sdk2 / sdk2py / ros2 / xr_teleoperate)
        │  rt/lowcmd
        ▼
  DDS bridge (this repo)
        │  PD: tau = tau_ff + kp*(q* - q) + kd*(dq* - dq)
        ▼
     MuJoCo physics
        │
        ▼
  rt/lowstate (+ sportmodestate, wireless, inspire/state)
```

---

## 1. Quick start — run the sim

### Option A: C++ simulator (recommended)

```bash
cd simulate
mkdir -p build && cd build
cmake .. && make -j4

# Use simulate/config.yaml
./unitree_mujoco

# Or override robot / scene / DDS
./unitree_mujoco -r go2 -s scene_terrain.xml
./unitree_mujoco -r g1 -s scene_29dof.xml
./unitree_mujoco -r g1 -s scene_29dof.xml -i 1 -n lo
```

CLI flags: `-r` robot, `-s` scene, `-i` domain id, `-n` interface.

### Option B: Python simulator

Edit `simulate_python/config.py`, then:

```bash
cd simulate_python
python3 unitree_mujoco.py
```

In another terminal, smoke-test DDS:

```bash
python3 test/test_unitree_sdk2.py      # Go2 / unitree_go
python3 test/test_unitree_sdk2_g1.py   # G1 / unitree_hg
```

### Option C: G1 + Inspire teleop sim (sim2sim with Quest)

```bash
cd simulate_python
python g1_inspire_sim.py                 # viewer + DDS domain 1
# python g1_inspire_sim.py --headless    # no viewer
# python g1_inspire_sim.py --no-camera   # skip head cameras
```

See [G1_INSPIRE_TELEOP.md](./G1_INSPIRE_TELEOP.md) for the full Quest / xr_teleoperate stack.

### Humanoid tip (elastic band)

For H1/G1 standing init, enable the virtual hoist (`enable_elastic_band`), then in the viewer:

| Key | Action |
|-----|--------|
| **9** | Toggle elastic band on/off |
| **7** | Lower robot |
| **8** | Lift robot |

Lower onto the feet before releasing the band.

---

## 2. Parameters

### Sim vs real (the important pair)

| Mode | Domain ID | Interface |
|------|-----------|-----------|
| **Simulation** | `1` | `"lo"` (loopback) |
| **Real robot** | `0` | robot Ethernet NIC (e.g. `enp3s0`) |

Keep controller logic identical; only change how you initialize the DDS channel factory.

### C++ — `simulate/config.yaml`

| Parameter | Example | Meaning |
|-----------|---------|---------|
| `robot` | `"g1"` | Folder under `unitree_robots/` |
| `robot_scene` | `"scene_29dof.xml"` | Scene XML in that folder |
| `domain_id` | `1` | DDS domain (`1` sim, `0` real) |
| `interface` | `"lo"` | DDS network interface |
| `use_joystick` | `0` / `1` | Publish gamepad as `rt/wirelesscontroller` |
| `joystick_type` | `"xbox"` / `"switch"` | Button/axis layout |
| `joystick_device` | `"/dev/input/js0"` | Joystick device path |
| `joystick_bits` | `16` | Axis resolution |
| `print_scene_information` | `1` | Dump links / joints / sensors at start |
| `enable_elastic_band` | `1` | Virtual hoist for humanoids |

Supported robots include: `go2`, `go2w`, `b2`, `b2w`, `a2`, `h1`, `h1_2`, `h2`, `g1`, `r1`.

### Python — `simulate_python/config.py`

| Parameter | Example | Meaning |
|-----------|---------|---------|
| `ROBOT` | `"go2"` | Robot name |
| `ROBOT_SCENE` | `../unitree_robots/{ROBOT}/scene.xml` | Full path to scene |
| `DOMAIN_ID` | `1` | DDS domain |
| `INTERFACE` | `"lo"` | DDS interface |
| `USE_JOYSTICK` | `1` | Gamepad → wireless controller |
| `JOYSTICK_TYPE` | `"xbox"` | Layout |
| `JOYSTICK_DEVICE` | `0` | pygame joystick index |
| `PRINT_SCENE_INFORMATION` | `True` | Scene dump |
| `ENABLE_ELASTIC_BAND` | `False` | Virtual hoist |
| `SIMULATE_DT` | `0.005` | Physics timestep (s); must exceed `viewer.sync()` cost |
| `VIEWER_DT` | `0.02` | Viewer refresh (~50 Hz) |

**Note:** C++ `config.yaml` may be set to G1 while Python `config.py` defaults to Go2 — set the robot explicitly before you run.

### G1 + Inspire teleop (`g1_inspire_sim.py` / `g1_inspire_bridge.py`)

| Parameter | Typical | Meaning |
|-----------|---------|---------|
| Domain | `1` | Matches `xr_teleoperate --sim` |
| `SIMULATE_DT` | `0.002` | Physics dt |
| Hold gains | `kp=60`, `kd=1.5` | Hold uncommanded joints until first `lowcmd` |
| Inspire cmds | normalized `0–1` | `1.0` open, `0.0` closed |

Camera stream (optional): `simulate_python/cam_config_mujoco.yaml`.

### Control message types

| Robots | IDL |
|--------|-----|
| Go2, B2, H1, Go2w, B2w, … | `unitree_go` |
| G1, H1-2 | `unitree_hg` |

Low-level topics used here:

- `rt/lowcmd` / `rt/lowstate`
- `rt/sportmodestate` (pose/vel kept in sim even when real robot hides it)
- `rt/wirelesscontroller`
- `rt/secondary_imu` (G1, C++ bridge)
- G1+Inspire: `rt/inspire/cmd`, `rt/inspire/state`

Motor index order matches hardware. For G1, see `unitree_robots/g1/g1_joint_index_dds.md`.

### Terrain

Parametric stairs / rough ground / height maps: see `terrain_tool/readme.md`. Output scenes (e.g. `scene_terrain.xml`) are loaded via `robot_scene` / `-s`.

This repo does **not** implement runtime domain randomization or RL policy export.

---

## 3. Sim → Real

Same LowCmd / LowState API; only DDS transport changes.

### Checklist

1. Develop and verify the controller against MuJoCo with `domain_id=1`, `interface=lo`.
2. Confirm motor indices, PD gains, and command rate match hardware docs.
3. On the real robot: turn off conflicting onboard motion services if you take over low-level control.
4. Re-run the **same** controller with `domain_id=0` and the robot NIC name.

### Python example (`example/python/stand_go2.py`)

```bash
# Terminal 1 — start Go2 sim (config: robot=go2, domain 1, lo)
cd simulate/build && ./unitree_mujoco -r go2 -s scene.xml

# Terminal 2 — controller → sim
cd example/python
python3 stand_go2.py

# Same controller → real robot
python3 stand_go2.py enp3s0   # replace with your NIC
```

```python
if len(sys.argv) < 2:
    ChannelFactoryInitialize(1, "lo")       # sim
else:
    ChannelFactoryInitialize(0, sys.argv[1])  # real
```

### C++ example (`example/cpp/stand_go2`)

```bash
cd example/cpp && mkdir -p build && cd build && cmake .. && make -j4
./stand_go2            # sim
./stand_go2 enp3s0     # real
```

### ROS2 example (`example/ros2`)

```bash
# Sim
source ~/unitree_ros2/setup_local.sh
export ROS_DOMAIN_ID=1
./install/stand_go2/bin/stand_go2

# Real
source ~/unitree_ros2/setup.sh
export ROS_DOMAIN_ID=0
./install/stand_go2/bin/stand_go2
```

### G1 + Inspire teleop → real

Planned path (controller stays the same):

1. Drop `--sim` on `xr_teleoperate`.
2. Use DDS domain `0` + robot network interface.
3. Ensure Inspire hand DDS and body `unitree_hg` topics match the physical stack.

Locomotion RL deploy lives in a separate `deploy/` repo, not here.

---

## 4. Sim → Sim

Use this MuJoCo backend as a drop-in for another simulator (e.g. Isaac Lab) when the controller already speaks Unitree DDS.

### Pattern

| Target | Domain | Interface | Notes |
|--------|--------|-----------|-------|
| This MuJoCo sim | `1` | `lo` (or LAN NIC if teleop is remote) | Start `unitree_mujoco` or `g1_inspire_sim.py` |
| Isaac / other Unitree sim | usually `1` + `--sim` | as that stack documents | Same topics |
| Real robot | `0` | robot NIC | See section 3 |

Any controller that publishes `rt/lowcmd` (and optionally inspire / wireless topics) can target MuJoCo without code changes beyond channel init.

### G1 + Inspire: Isaac Lab → MuJoCo

Documented end-to-end in [G1_INSPIRE_TELEOP.md](./G1_INSPIRE_TELEOP.md):

```
Quest 3 → xr_teleoperate (--sim) → DDS domain 1 → MuJoCo (g1_inspire_sim)
```

Compatibility choices:

- Same DDS topics/types as Isaac and the real robot
- Same Inspire 0–1 normalization as Isaac `inspire_dds.py`
- Fixed pelvis (upper body + hands only; no walking policy required)

```bash
# Terminal 1 — MuJoCo
conda activate unitree
cd simulate_python
python g1_inspire_sim.py

# Terminal 2 — teleop (external xr_teleoperate repo)
conda activate tv
cd ../xr_teleoperate/teleop
python teleop_hand_and_arm.py --arm=G1_29 --ee=inspire_dfx --sim
```

Standalone fake teleop (no Quest):

```bash
python g1_inspire_sim.py --headless   # terminal 1
python test/test_g1_inspire_teleop.py # terminal 2
```

### Generic controller sim2sim

1. Start MuJoCo with the matching robot/scene and domain `1`.
2. Point your existing Isaac/sdk2 controller at domain `1` + `lo`.
3. Verify joint order (`unitree_go` vs `unitree_hg`) and PD form.
4. Iterate gains/timing in MuJoCo before sim2real.

There is no ONNX / policy-export package in this repo — transfer is at the **DDS + PD command** level.

---

## 5. Key files

| Path | Role |
|------|------|
| `simulate/config.yaml` | C++ sim parameters |
| `simulate_python/config.py` | Python sim parameters |
| `simulate/build/unitree_mujoco` | C++ entry binary |
| `simulate_python/unitree_mujoco.py` | Python entry |
| `simulate_python/g1_inspire_sim.py` | G1+Inspire teleop sim |
| `simulate_python/g1_inspire_bridge.py` | Body + hand DDS bridge |
| `example/python/stand_go2.py` | Sim2real pattern (Python) |
| `example/cpp/stand_go2.cpp` | Sim2real pattern (C++) |
| `example/ros2/` | Sim2real pattern (ROS2) |
| `unitree_robots/` | MJCF scenes per robot |
| `terrain_tool/` | Procedural terrain scenes |

---

## 6. Related docs

| Doc | Contents |
|-----|----------|
| [readme.md](./readme.md) | Install, overview, joystick, terrain, sim2real examples |
| [G1_INSPIRE_TELEOP.md](./G1_INSPIRE_TELEOP.md) | Quest → MuJoCo sim2sim teleop |
| [PROGRESS.md](./PROGRESS.md) | Session notes / camera + image server |
| [terrain_tool/readme.md](./terrain_tool/readme.md) | Terrain generator |
| [unitree_robots/g1/g1_joint_index_dds.md](./unitree_robots/g1/g1_joint_index_dds.md) | G1 joint DDS order |
