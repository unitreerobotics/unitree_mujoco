# Progress — G1 + Inspire Teleop in MuJoCo (Quest 3)

Last updated: 2026-07-20

## Done

### 1. Robot model (Phase 0)
- Converted Inspire hand URDFs to MJCF (`unitree_robots/g1/inspire_build/convert_hands.py`).
- Merged both hands into the G1 29-DoF model (`inspire_build/merge_g1_inspire.py`):
  - Hands attached to `left/right_wrist_yaw_link`, original rubber hands removed.
  - Pelvis fixed (freejoint removed) — robot stands still, upper-body teleop only.
  - URDF `mimic` joints converted to MuJoCo `<equality>` constraints.
  - 12 position actuators added for the hands (6 per hand).
- Output: `unitree_robots/g1/g1_29dof_inspire_fixed.xml` + `scene_29dof_inspire_fixed.xml`.

### 2. DDS bridge (Phases 1–2)
- `simulate_python/g1_inspire_bridge.py`:
  - Body: subscribes `rt/lowcmd`, publishes `rt/lowstate` (joints + IMU), PD control in MuJoCo.
  - Hands: subscribes `rt/inspire/cmd`, publishes `rt/inspire/state`, with the same
    normalization ranges as `unitree_sim_isaaclab` so `xr_teleoperate` works unmodified.
  - Default PD gains hold the legs/waist so the robot stands stably with no commands.
- `simulate_python/g1_inspire_sim.py`: sim launcher, DDS domain 1 (= `xr_teleoperate --sim`),
  physics + viewer threads, `--headless` option.

### 3. Automated control test (Phase 3)
- `simulate_python/test/test_g1_inspire_teleop.py`: fake teleoperator that sends sinusoidal
  arm commands + hand open/close over DDS and verifies `rt/lowstate` / `rt/inspire/state`
  feedback. **Passing**.

### 4. Camera feed to Quest 3 (Phase 4) — done, verified end-to-end
Same pipeline as Isaac Sim: sim renders → shared memory → teleimager image server →
`xr_teleoperate` / Quest (ZMQ :55555 / WebRTC :60001, config served on :60000).

- Stereo head cameras added to the model (`head_left_eye` / `head_right_eye`):
  IPD 64 mm, mounted on the head front, pitched 25° down, fovy 70°,
  1280x720 offscreen framebuffer (`inspire_build/merge_g1_inspire.py`).
- `g1_inspire_sim.py` got a `CameraThread`: renders both eyes at 480x640 @ 30 FPS via EGL
  (`MUJOCO_GL=egl` set automatically) and writes them to shared memory
  (`isaac_left/right_image_shm`) in the exact Isaac format. `--no-camera` disables it.
- `simulate_python/tools/shared_memory_utils.py`: writer/reader copied from
  `unitree_sim_isaaclab` (self-contained, no cross-repo import).
- `simulate_python/run_image_server.py` + `cam_config_mujoco.yaml`: launches teleimager's
  IsaacSim-mode image server (runs in the `tv` env). Head camera binocular 480x1280,
  wrist cameras disabled.
- Environment fixes:
  - `xr_teleoperate/teleop/teleimager/.../image_server.py`: `logging_mp.basicConfig` →
    `basic_config` (import crashed in the `tv` env).
  - Installed `aiortc` 1.15.0 into the `tv` env (WebRTC support; was missing).
  - Generated a self-signed TLS cert at `~/.config/xr_teleoperate/{cert,key}.pem`
    (used by both televuer and teleimager WebRTC).

**End-to-end test passed:** headless sim + image server running together —
`ImageClient` received the camera config on :60000 and a live 480x1280 binocular frame
over ZMQ (first-person view, both Inspire hands visible). The DDS control test still
passes while the camera streams (no interference between the two paths).

## How to run the full Quest 3 session

```bash
# Terminal 1 — MuJoCo sim (unitree env)
cd unitree_mujoco/simulate_python
python g1_inspire_sim.py

# Terminal 2 — image server (tv env)
cd unitree_mujoco/simulate_python
conda activate tv
python run_image_server.py

# Terminal 3 — teleop (tv env)
cd xr_teleoperate/teleop
conda activate tv
python teleop_hand_and_arm.py --arm=G1_29 --ee=inspire_dfx --sim \
    --img-server-ip 127.0.0.1 --image-transport zmq
```

Then on the Quest 3 browser open `https://<PC-IP>:8012?ws=wss://<PC-IP>:8012`,
accept the self-signed cert, enter VR, and press **r** in Terminal 3 to start teleop.
(`--image-transport zmq` is the simplest; for WebRTC, first visit
`https://<PC-IP>:60001` once on the Quest to accept the cert, then use
`--image-transport webrtc`.)

## Known limitations
- Robot base is fixed (no locomotion) — intended for upper-body + hands teleop.
- Scene is an empty floor; no table/objects yet.
- Head cameras are fixed to the torso (no head-yaw follow).
