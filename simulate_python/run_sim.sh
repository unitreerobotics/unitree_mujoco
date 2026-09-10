#!/usr/bin/env bash
# Launch the MuJoCo simulator on macOS using mjpython.
# mjpython is required because macOS needs GUI operations on the main thread.
#
# Usage: ./run_sim.sh

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
VENV_DIR="$SCRIPT_DIR/../.venv"

if [ ! -f "$VENV_DIR/bin/mjpython" ]; then
    echo "Error: mjpython not found. Set up the venv first:"
    echo "  cd $(dirname "$SCRIPT_DIR") && uv venv --python 3.11 .venv && uv pip install mujoco pygame numpy"
    exit 1
fi

# mjpython needs libpython on DYLD_LIBRARY_PATH when using uv-managed Python
PYTHON_LIBDIR="$("$VENV_DIR/bin/python3" -c 'import sysconfig; print(sysconfig.get_config_var("LIBDIR"))')"
export DYLD_LIBRARY_PATH="$PYTHON_LIBDIR${DYLD_LIBRARY_PATH:+:$DYLD_LIBRARY_PATH}"

cd "$SCRIPT_DIR"
exec "$VENV_DIR/bin/mjpython" unitree_mujoco.py "$@"
