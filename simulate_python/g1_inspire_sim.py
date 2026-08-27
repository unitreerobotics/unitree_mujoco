"""G1 29-DoF + Inspire hands MuJoCo sim with a Unitree DDS bridge.

Counterpart of unitree_sim_isaaclab for the xr_teleoperate stack:

    Terminal 1:  python g1_inspire_sim.py
    Terminal 2:  python run_image_server.py            (tv env, for Quest video)
    Terminal 3:  cd xr_teleoperate/teleop && \
                 python teleop_hand_and_arm.py --arm=G1_29 --ee=inspire_dfx --sim

Uses DDS domain 1 (simulation), same as xr_teleoperate --sim.
Head-camera frames (left/right eye) are written to shared memory in the same
format as unitree_sim_isaaclab, so teleimager's IsaacSim image server can
stream them to the Quest.
""" 

import argparse
import os
import threading
import time
from pathlib import Path
from threading import Thread

# Offscreen head-camera rendering uses EGL so it does not fight over GLFW with
# the interactive viewer (which always uses GLFW, independent of MUJOCO_GL).
# Must be set before mujoco is imported.
os.environ.setdefault("MUJOCO_GL", "egl")

import mujoco
import mujoco.viewer

from unitree_sdk2py.core.channel import ChannelFactoryInitialize
from g1_inspire_bridge import G1InspireBridge

SCENE = str(Path(__file__).resolve().parents[1] / "unitree_robots/g1/scene_29dof_inspire_fixed.xml")
DOMAIN_ID = 1  # 1 = simulation (matches xr_teleoperate --sim), 0 = real robot
SIMULATE_DT = 0.002
VIEWER_DT = 0.02
CAMERA_FPS = 30
EYE_HEIGHT, EYE_WIDTH = 480, 640  # per eye; binocular head stream = 480x1280

locker = threading.Lock()


def CameraThread(mj_model, mj_data, is_running):
    """Render the stereo head cameras and write frames to shared memory."""
    from tools.shared_memory_utils import MultiImageWriter

    try:
        renderer = mujoco.Renderer(mj_model, height=EYE_HEIGHT, width=EYE_WIDTH)
    except Exception as e:
        print(f"[g1_inspire_sim] camera renderer failed ({e}); "
              "continuing without camera (use --no-camera to silence)")
        return
    # MuJoCo renders RGB; writer converts RGB->BGR by default (skip_cvtcolor=False)
    writer = MultiImageWriter()
    period = 1.0 / CAMERA_FPS
    print(f"[g1_inspire_sim] head cameras streaming to shared memory @ {CAMERA_FPS} FPS")
    try:
        while is_running():
            t0 = time.perf_counter()
            images = {}
            with locker:
                for key, cam in (("left", "head_left_eye"), ("right", "head_right_eye")):
                    renderer.update_scene(mj_data, camera=cam)
                    images[key] = renderer.render()
            writer.write_images(images)
            remain = period - (time.perf_counter() - t0)
            if remain > 0:
                time.sleep(remain)
    finally:
        writer.close()
        renderer.close()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--interface", type=str, default="lo",
                        help="network interface for DDS (default: lo)")
    parser.add_argument("--scene", type=str, default=SCENE)
    parser.add_argument("--headless", action="store_true",
                        help="run without the MuJoCo viewer (for testing)")
    parser.add_argument("--no-camera", action="store_true",
                        help="disable head-camera rendering to shared memory")
    args = parser.parse_args()

    mj_model = mujoco.MjModel.from_xml_path(args.scene)
    mj_data = mujoco.MjData(mj_model)
    mj_model.opt.timestep = SIMULATE_DT
    stop_event = threading.Event()

    if args.headless:
        class _FakeViewer:
            def is_running(self):
                return not stop_event.is_set()

            def sync(self):
                pass

        viewer = _FakeViewer()
    else:
        viewer = mujoco.viewer.launch_passive(mj_model, mj_data)

    ChannelFactoryInitialize(DOMAIN_ID, args.interface)
    bridge = G1InspireBridge(mj_model, mj_data, locker)
    print(f"[g1_inspire_sim] DDS bridge up (domain {DOMAIN_ID}, interface {args.interface})")
    print("[g1_inspire_sim] topics: rt/lowcmd rt/lowstate rt/inspire/cmd rt/inspire/state")

    def SimulationThread():
        while not stop_event.is_set() and viewer.is_running():
            step_start = time.perf_counter()
            with locker:
                bridge.update_ctrl()
                mujoco.mj_step(mj_model, mj_data)
            remain = mj_model.opt.timestep - (time.perf_counter() - step_start)
            if remain > 0:
                time.sleep(remain)

    def ViewerThread():
        while not stop_event.is_set() and viewer.is_running():
            with locker:
                viewer.sync()
            time.sleep(VIEWER_DT)

    sim_thread = Thread(target=SimulationThread)
    viewer_thread = Thread(target=ViewerThread)
    threads = [sim_thread, viewer_thread]
    if not args.no_camera:
        threads.append(Thread(
            target=CameraThread,
            args=(mj_model, mj_data,
                  lambda: not stop_event.is_set() and viewer.is_running()),
            daemon=True,
        ))
    for t in threads:
        t.start()
    try:
        sim_thread.join()
        viewer_thread.join()
    except KeyboardInterrupt:
        pass
    finally:
        stop_event.set()
        sim_thread.join(1.0)
        viewer_thread.join(1.0)
        bridge.close()
        if not args.headless:
            viewer.close()


if __name__ == "__main__":
    main()
