# R1 Gestures Quick Reference Card

## Installation
```bash
# Install dependencies
pip3 install mujoco pygame numpy

# Ensure config is set
cd simulate_python
# Edit config.py: ROBOT = "r1"
```

## Quick Commands
```bash
# Generate gesture files
python3 example_r1_gestures.py generate

# Play all gestures
python3 example_r1_gestures.py replay

# Record gestures interactively
python3 example_r1_gestures.py record
```

## Interactive Recording Controls
| Key | Action |
|-----|--------|
| **r** | Start/stop recording |
| **s** | Save recording |
| **c** | Clear current recording |
| **q** | Quit |

## Available Gestures
| Name | Duration | Type |
|------|----------|------|
| wave_right | 3.0s | Waves right arm |
| wave_left | 3.0s | Waves left arm |
| handshake_right | 2.0s | Right arm handshake |
| handshake_left | 2.0s | Left arm handshake |
| heart | 3.0s | Both arms heart shape |

## Python API Examples

### Load & Play
```python
from motion_capture import MotionCapture
from motion_replay import MotionPlayer

motion = MotionCapture(35, 0.002)
motion.load_motion("data/gestures/wave_right.json")

player = MotionPlayer(35)
player.load_motion(motion)
player.start_playback(loops=3, speed=1.5)

while player.is_playing():
    frame = player.step(0.002)
    # Send frame.joint_positions to robot
```

### Record Motion
```python
from motion_capture import MotionCapture

motion = MotionCapture(35, 0.002)
motion.start_recording()

# In simulation loop
motion.capture_frame(time, positions, velocities, kp_gains, kd_gains)

motion.stop_recording()
motion.save_motion("my_gesture.json", {"name": "custom"})
```

### Sequence Multiple Gestures
```python
from motion_replay import MotionSequence

seq = MotionSequence(35)
seq.add_motion(wave_motion, loops=1, speed=1.0)
seq.add_motion(heart_motion, loops=2, speed=0.8)
seq.start()

while not seq.is_finished():
    frame = seq.step(0.002)
```

### Interpolate Frames
```python
# Get exact position at any time
frame = motion.interpolate_frame(1.234)
positions = np.array(frame.joint_positions)
```

## R1 Motor Map

```
Legs (12 DOF):
  0-5: left leg (hip_pitch, hip_roll, hip_yaw, knee, ankle_pitch, ankle_roll)
  6-11: right leg (same structure)

Torso (2 DOF):
  12: waist_roll
  13: waist_yaw

Left Arm (6 DOF):
  14: shoulder_pitch
  15: shoulder_roll
  16: shoulder_yaw
  17: elbow
  18: wrist_roll
  19: wrist_roll_2

Right Arm (6 DOF):
  20: shoulder_pitch
  21: shoulder_roll
  22: shoulder_yaw
  23: elbow
  24: wrist_roll
  25: wrist_roll_2

Head (2 DOF):
  26: head_pitch
  27: head_yaw

Reserved (7 DOF):
  28-34: (future use)
```

## Playback Options

### Speed Control
```python
player.start_playback(loops=1, speed=0.5)   # Half speed
player.start_playback(loops=1, speed=1.0)   # Normal
player.start_playback(loops=1, speed=2.0)   # Double speed
```

### Looping
```python
player.start_playback(loops=1)    # Play once
player.start_playback(loops=5)    # Play 5 times
player.start_playback(loops=0)    # Infinite loop
```

### Callbacks
```python
def motion_complete():
    print("Finished!")

player.on_loop_complete = motion_complete
player.start_playback(loops=3)
```

## File Locations

```
simulate_python/
  ├── motion_capture.py      (Recording system)
  ├── motion_replay.py       (Playback engine)
  ├── r1_gestures.py         (Gesture definitions)
  └── test/
      └── test_r1_gestures.py (Test suite)

example/python/
  └── example_r1_gestures.py (User example)

doc/
  └── R1_GESTURES.md         (Full documentation)

data/
  └── gestures/              (Generated gesture files)
```

## Troubleshooting

| Problem | Solution |
|---------|----------|
| ImportError: motion_capture | Run from `example/python` or set PYTHONPATH |
| No viewer window | Install pygame: `pip3 install pygame` |
| Motor mismatch error | Check `config.py`: `ROBOT = "r1"` |
| Recording not working | Ensure viewer has keyboard focus |
| Gesture looks jerky | Reduce playback speed or use interpolation |

## Common Tasks

**Generate all gestures:**
```bash
python3 example_r1_gestures.py generate
```

**Play specific gesture 5 times at 1.5x speed:**
```python
motion = MotionCapture(35, 0.002)
motion.load_motion("data/gestures/wave_right.json")
player = MotionPlayer(35)
player.load_motion(motion)
player.start_playback(loops=5, speed=1.5)
```

**Record custom gesture and save:**
```bash
python3 example_r1_gestures.py record
# Press r to start, perform gesture, press r to stop
# Press s to save
```

**Chain multiple gestures:**
```python
seq = MotionSequence(35)
for gesture_file in ["wave_right.json", "heart.json"]:
    m = MotionCapture(35, 0.002)
    m.load_motion(f"data/gestures/{gesture_file}")
    seq.add_motion(m, loops=2, speed=1.0)
seq.start()
```

## Support Files

- **QUICKSTART_GESTURES.md** - 3-step setup guide
- **doc/R1_GESTURES.md** - Comprehensive documentation
- **test/test_r1_gestures.py** - Test suite (run for verification)

## Key Parameters

| Parameter | Type | Range | Default |
|-----------|------|-------|---------|
| loops | int | 0+ (0=infinite) | 1 |
| speed | float | 0.1 - 10.0 | 1.0 |
| timestep | float | seconds | 0.002 |
| num_motors | int | fixed | 35 |

---

For full documentation, see `doc/R1_GESTURES.md`
