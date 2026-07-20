"""Image server for the MuJoCo G1+Inspire sim -> Quest 3.

Streams the head-camera frames that g1_inspire_sim.py writes to shared memory,
reusing teleimager's IsaacSim mode (ZMQ on 55555, WebRTC on 60001, camera
config served on 60000).

Run in the *tv* conda env (where teleimager is installed):

    conda activate tv
    python run_image_server.py

Start g1_inspire_sim.py first so shared-memory frames exist.
"""

import os
import signal
import sys

import yaml

# Make `tools.shared_memory_utils` importable for teleimager's IsaacSimCamera.
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from teleimager.image_server import ImageServer, signal_handler

CONFIG = os.path.join(os.path.dirname(os.path.abspath(__file__)), "cam_config_mujoco.yaml")


def main():
    with open(CONFIG, "r") as f:
        cam_config = yaml.safe_load(f)

    server = ImageServer(cam_config, realsense_enable=False,
                         camera_finder_verbose=False, isaacsim_enable=True)
    signal.signal(signal.SIGINT, lambda s, f: signal_handler(server, s, f))
    signal.signal(signal.SIGTERM, lambda s, f: signal_handler(server, s, f))
    server.start()
    print("[run_image_server] streaming MuJoCo head camera "
          "(config :60000, zmq :55555, webrtc :60001). Ctrl+C to stop.")
    server.wait()


if __name__ == "__main__":
    main()
