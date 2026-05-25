# Quick Start: R1 Gestures in MuJoCo

This guide gets you started with recording and replaying robot gestures in just 3 steps.

## What You Can Do

- 🎬 **Record gestures** - Capture custom robot motions by hand or programmatically
- ⏯️ **Play gestures** - Replay recorded motions with loop and speed control
- 🎨 **Pre-built gestures** - Use included wave, handshake, and heart gestures
- 🔗 **Chain gestures** - Sequence multiple gestures together

## Prerequisites

```bash
# Install Python dependencies
pip3 install mujoco pygame numpy

# Ensure config.py is set to R1
cd simulate_python
# Edit config.py: set ROBOT = "r1"
```

## Step 1: Generate Gestures

Generate motion files for all pre-scripted gestures:

```bash
cd example/python
python3 example_r1_gestures.py generate
```

Output: Creates `data/gestures/*.json` files

## Step 2: Replay Gestures

Play all gestures in sequence in the MuJoCo simulator:

```bash
python3 example_r1_gestures.py replay
```

The viewer opens and R1 performs:
1. Wave right (2x)
2. Wave left (2x)
3. Handshake right (2x)
4. Handshake left (2x)
5. Heart shape (2x)

Controls:
- Mouse drag to rotate view
- Scroll to zoom
- Press 'q' to quit

## Step 3: Record Custom Gestures

Record your own gestures interactively:

```bash
python3 example_r1_gestures.py record
```

Controls:
- **r** - Start/stop recording
- **s** - Save recording
- **c** - Clear current recording
- **q** - Quit

Drag robot joints in the viewer to create your motion. Press 'r' to start recording, perform your gesture, press 'r' again to stop.

## Using in Your Code

```python
from motion_capture import MotionCapture
from motion_replay import MotionPlayer
import numpy as np

# Load a gesture
motion = MotionCapture(35, 0.002)
motion.load_motion("data/gestures/wave_right.json")

# Create player
player = MotionPlayer(35)
player.load_motion(motion)

# Play 3 times at normal speed
player.start_playback(loops=3, speed=1.0)

# In your simulation loop
while player.is_playing():
    frame = player.step(dt)
    if frame:
        positions = np.array(frame.joint_positions)
        # Send positions to robot
```

## Available Gestures

| Gesture | Duration | Loop |
|---------|----------|------|
| wave_right | 3.0s | Waves right hand |
| wave_left | 3.0s | Waves left hand |
| handshake_right | 2.0s | Handshake with right arm |
| handshake_left | 2.0s | Handshake with left arm |
| heart | 3.0s | Forms heart shape |

## Troubleshooting

### ImportError: No module named 'motion_capture'
Run scripts from `example/python` directory and ensure python paths are set correctly.

### "ROBOT = r1" not found in config
Edit `simulate_python/config.py` and set `ROBOT = "r1"`

### No viewer window appears
Install pygame: `pip3 install pygame`

## Next Steps

- **Edit gestures** - Load JSON files and modify joint positions
- **Create custom gestures** - Subclass `R1Gesture` in `r1_gestures.py`
- **Build apps** - Use motion player API in your control programs

See `doc/R1_GESTURES.md` for detailed documentation.

---

Ready to start? Run:
```bash
cd example/python
python3 example_r1_gestures.py generate
python3 example_r1_gestures.py replay
```
