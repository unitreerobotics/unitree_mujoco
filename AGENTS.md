# AI Agent Instructions for unitree_mujoco

## What this repository is
- A simulator for Unitree robots based on MuJoCo.
- Two simulator implementations:
  - `simulate/`: C++ simulator using `unitree_sdk2` (recommended)
  - `simulate_python/`: Python simulator using `unitree_sdk2_python`
- Includes robot MJCF scene files under `unitree_robots/` and terrain generation tools in `terrain_tool/`.
- Example programs in `example/`, including `example/python/stand_go2.py` for simulation and physical robot control.

## Primary tasks agents should support
- Fixing and extending simulation code in `simulate/src/` and `simulate_python/`
- Updating `simulate/config.yaml`, `simulate_python/config.py`, and robot scene integration
- Improving tests, examples, and simulator behavior consistency
- Keeping the C++ and Python implementations aligned when both exist

## Key directories and entrypoints
- `simulate/`: C++ build, simulation executable, and native Unitree SDK bridge
  - key files: `simulate/src/main.cc`, `simulate/src/unitree_sdk2_bridge.h`, `simulate/src/unitree_sdk2_bridge/`
  - config: `simulate/config.yaml`
- `simulate_python/`: Python simulator and Python SDK bridge
  - key files: `simulate_python/unitree_mujoco.py`, `simulate_python/unitree_sdk2py_bridge.py`, `simulate_python/config.py`
- `example/python/stand_go2.py`: sample control program for simulation and physical robots
- `unitree_robots/`: robot scene XML assets and supported robot models
- `terrain_tool/`: procedural terrain generator and related docs

## Build and test commands
- C++ simulator:
  - install deps: `sudo apt install libyaml-cpp-dev libspdlog-dev libboost-all-dev libglfw3-dev`
  - install `unitree_sdk2` to `/opt/unitree_robotics`
  - build: `cd simulate && mkdir build && cd build && cmake .. && make -j4`
  - run: `./unitree_mujoco -r go2 -s scene_terrain.xml`
  - test binary: `./test`
- Python simulator:
  - install `unitree_sdk2_python` with `pip3 install -e .`
  - install MuJoCo and joystick support: `pip3 install mujoco pygame`
  - run: `cd simulate_python && python3 ./unitree_mujoco.py`
  - tests: `python3 ./test/test_unitree_sdk2.py`
- Example real robot control:
  - `cd example/python && python3 ./stand_go2.py enp3s0`

## Important conventions
- `readme.md` is the authoritative source for setup, usage, and dependency instructions.
- `simulate/config.yaml` and `simulate_python/config.py` control robot selection, scene path, DDS domain, network interface, joystick support, and elastic band behavior.
- Use `lo` for simulation network interface. Physical robot control uses the actual interface name.
- Supported robot names include `go2`, `b2`, `b2w`, `h1`, plus `g1` and `h1_2` variants.
- Joystick layout support is implemented for `xbox` and `switch`; update both C++ and Python bridge mappings if layouts change.
- Do not assume external dependencies are vendored; `unitree_sdk2`, `unitree_sdk2_python`, MuJoCo, and `cyclonedds` are installed externally.

## Common pitfalls for changes
- Avoid unintended changes to the MJCF/scene XML structure in `unitree_robots/`.
- Ensure native C++ builds reference `unitree_sdk2` installed under `/opt/unitree_robotics`.
- Keep C++ and Python behavior aligned for the same robot models and DDS message families.
- Use the correct IDL family: `unitree_go` for Go2/B2/H1/Go2w/B2w and `unitree_hg` for G1/H1_2.
- Python simulator failures often trace to missing `cyclonedds`, incorrect MuJoCo installation, or wrong `simulate_python/config.py` settings.

## Where to look first
- `readme.md`
- `simulate/src/unitree_sdk2_bridge/`
- `simulate_python/unitree_sdk2py_bridge.py`
- `simulate_python/config.py`
- `example/python/stand_go2.py`
- `unitree_robots/`

## When to ask for help
- If a change affects both the C++ and Python simulator paths, verify both build/test flows.
- If adding or modifying a robot model, confirm the correct `unitree_go` vs `unitree_hg` message family and DDS mapping.
