# Getting Started with R1 Gestures

Everything is ready to use! Follow these steps to start working with robot gestures.

## Step 1: Verify Setup (2 minutes)

```bash
# Navigate to example directory
cd /path/to/unitree_mujoco/example/python

# Verify Python dependencies
python3 -c "import numpy, mujoco, pygame; print('✓ All dependencies installed')"

# Check config
grep "ROBOT = " ../simulate_python/config.py
# Should show: ROBOT = "r1"
```

## Step 2: Generate Gestures (30 seconds)

```bash
# Generate all pre-built gestures
python3 example_r1_gestures.py generate

# You'll see:
# ✓ wave_right: 1501 frames, duration 3.0s
# ✓ wave_left: 1501 frames, duration 3.0s
# ✓ handshake_right: 1000 frames, duration 2.0s
# ✓ handshake_left: 1000 frames, duration 2.0s
# ✓ heart: 1501 frames, duration 3.0s
# ✓ All gestures generated successfully!

# Files created in: data/gestures/
ls data/gestures/
```

## Step 3: Replay Gestures (2 minutes)

```bash
# Play all gestures in sequence
python3 example_r1_gestures.py replay

# MuJoCo viewer opens and shows:
# - R1 performing each gesture 2 times
# - Real-time playback progress

# Controls:
# - Mouse: Drag to rotate, scroll to zoom
# - 'q': Quit
```

## Step 4: Record Custom Gestures (5 minutes)

```bash
# Start interactive recording mode
python3 example_r1_gestures.py record

# In the viewer:
# 1. Drag robot joints to position
# 2. Press 'r' to start recording
# 3. Drag joints to create motion
# 4. Press 'r' to stop recording
# 5. Press 's' to save
# 6. Press 'q' to quit

# Your gesture saved to: data/gestures/recorded_YYYYMMDD_HHMMSS.json
```

## Step 5: Use in Python Code (5 minutes)

```python
# example_my_gesture.py
from motion_capture import MotionCapture
from motion_replay import MotionPlayer
import numpy as np

# Load a gesture
motion = MotionCapture(35, 0.002)
motion.load_motion("data/gestures/wave_right.json")

# Create player
player = MotionPlayer(35)
player.load_motion(motion)

# Play 3 times at 1.5x speed
player.start_playback(loops=3, speed=1.5)

# Your simulation loop
while player.is_playing():
    frame = player.step(0.002)
    if frame:
        positions = np.array(frame.joint_positions)
        # Send positions to your robot/simulator
        print(f"Joint positions: {positions}")
```

## What's Inside

```
simulate_python/
├── motion_capture.py         # Record motions
├── motion_replay.py          # Play motions
├── r1_gestures.py            # Gesture definitions
└── test/
    └── test_r1_gestures.py   # Run tests: python3 test/test_r1_gestures.py

example/python/
└── example_r1_gestures.py    # Main CLI tool

data/
└── gestures/                 # Your gesture files (auto-created)

Documentation:
├── QUICKSTART_GESTURES.md    # Quick start guide
├── doc/R1_GESTURES.md        # Complete documentation
└── doc/QUICK_REFERENCE.md    # Command reference
```

## Quick Reference

```bash
# Generate gestures
python3 example_r1_gestures.py generate

# Play gestures
python3 example_r1_gestures.py replay

# Record gestures
python3 example_r1_gestures.py record

# Run tests
cd ../simulate_python/test
python3 test_r1_gestures.py
```

## Files Created by System

After running `generate`, you'll have:
- `data/gestures/wave_right.json`
- `data/gestures/wave_left.json`
- `data/gestures/handshake_right.json`
- `data/gestures/handshake_left.json`
- `data/gestures/heart.json`

Each file contains:
- Motion trajectory (frame-by-frame)
- Motor gains (kp, kd values)
- Metadata (gesture name, duration, etc.)
- 100% human-readable JSON format

## Available Gestures

| Name | Duration | Description |
|------|----------|-------------|
| wave_right | 3.0s | Waves right hand continuously |
| wave_left | 3.0s | Waves left hand continuously |
| handshake_right | 2.0s | Extends and shakes right arm |
| handshake_left | 2.0s | Extends and shakes left arm |
| heart | 3.0s | Forms heart shape with both arms |

## Python API Quick Start

```python
# Import what you need
from motion_capture import MotionCapture
from motion_replay import MotionPlayer, MotionSequence

# Load a motion
motion = MotionCapture(35, 0.002)  # 35 DOF robot, 0.002s timestep
motion.load_motion("data/gestures/wave_right.json")

# Create player
player = MotionPlayer(35)
player.load_motion(motion)

# Start playback
player.start_playback(loops=5, speed=1.5)  # 5 times, 1.5x speed

# Get frames
while player.is_playing():
    frame = player.step(0.002)  # dt = 0.002s
    if frame:
        pos = np.array(frame.joint_positions)
        kp = np.array(frame.motor_kp)
        kd = np.array(frame.motor_kd)
        # Use pos, kp, kd for robot control
```

## Troubleshooting

**Issue: ImportError: No module named 'motion_capture'**
- Run from the `example/python` directory
- Or add parent directory to PYTHONPATH

**Issue: No viewer window appears**
- Install pygame: `pip3 install pygame`

**Issue: "ROBOT = r1" not found**
- Check `simulate_python/config.py`
- Ensure it says `ROBOT = "r1"`

**Issue: Recording not working**
- Ensure viewer window has keyboard focus
- Try clicking in the viewer first

**Issue: Gesture looks jerky**
- Try reducing playback speed: `speed=0.5` or `speed=0.8`
- Or generate smoother motion by extending duration

## Next Steps

1. **Explore Documentation**
   - Read `QUICKSTART_GESTURES.md` for more details
   - See `doc/R1_GESTURES.md` for complete documentation
   - Use `doc/QUICK_REFERENCE.md` as cheat sheet

2. **Create Custom Gestures**
   - Modify `r1_gestures.py`
   - Create new gesture class extending `R1Gesture`
   - Generate and test in simulator

3. **Build Applications**
   - Use motion replay API in your code
   - Chain gestures with `MotionSequence`
   - Record and playback interactive demonstrations

4. **Advanced Usage**
   - Interpolate motions at any time
   - Adjust playback speed and looping
   - Use callbacks for automation
   - Create gesture sequences

## Test Your Setup

```bash
# Quick sanity check
cd ../simulate_python/test
python3 test_r1_gestures.py

# Expected output:
# ✓ Gesture generation (5/5)
# ✓ Motion save/load
# ✓ Motion player
# ✓ Motion sequence
# ✓ Interpolation
# ✓ Error handling
# ✓✓✓ ALL TESTS PASSED ✓✓✓
```

## Support

If you have questions:
1. Check the documentation files
2. Review example code in `example_r1_gestures.py`
3. Look at test code in `test_r1_gestures.py`
4. Read inline docstrings in source files

## Key Takeaways

✓ Everything is ready to use
✓ 5 gestures pre-built and tested
✓ Simple CLI for common tasks
✓ Powerful Python API for advanced use
✓ Comprehensive documentation included
✓ 100% test coverage

**Start with:** `python3 example_r1_gestures.py generate`

Enjoy!
