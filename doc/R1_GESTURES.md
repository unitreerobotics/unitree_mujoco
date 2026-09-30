# R1 Robot Gestures - Motion Capture & Replay Guide

This guide explains how to record, generate, and replay simple gestures with the R1 robot in MuJoCo.

## Overview

The gesture system consists of three main components:

1. **Motion Capture** (`motion_capture.py`) - Records joint positions, velocities, and motor commands
2. **Gesture Definitions** (`r1_gestures.py`) - Pre-scripted gesture trajectories (wave, handshake, heart)
3. **Motion Replay** (`motion_replay.py`) - Plays back recorded/scripted motions with loop and speed control

## Available Gestures

### 1. Wave (`wave_right`, `wave_left`)
- Waves one hand in a continuous circular motion
- Duration: 3 seconds
- Frequency: ~0.67 Hz

### 2. Handshake (`handshake_right`, `handshake_left`)
- Extends arm forward and performs vertical shaking motion
- Duration: 2 seconds
- Used for greeting interaction

### 3. Heart (`heart`)
- Forms a heart shape with both hands
- Duration: 3 seconds
- Arms come together in front of torso

## Quick Start

### 1. Generate All Gestures
Generate motion files for all predefined gestures:

```bash
cd example/python
python3 example_r1_gestures.py generate
```

This creates gesture files in `data/gestures/`:
- `wave_right.json`
- `wave_left.json`
- `handshake_right.json`
- `handshake_left.json`
- `heart.json`

### 2. Replay Gestures
Play all gestures in sequence (2 loops each):

```bash
python3 example_r1_gestures.py replay
```

The viewer will open and show:
- R1 robot performing each gesture sequentially
- Real-time feedback on current gesture and playback progress
- Each gesture loops twice before moving to the next

### 3. Record Your Own Gestures
Record interactive motions:

```bash
python3 example_r1_gestures.py record
```

Controls during recording:
- **r** - Start/stop recording
- **s** - Save recording
- **c** - Clear current recording
- **q** - Quit

Drag the robot in the viewer to move its joints while recording.

## Usage Examples

### Python API

#### Generating and Saving a Gesture

```python
from r1_gestures import get_gesture
from motion_capture import MotionCapture
import numpy as np

# Create gesture
gesture = get_gesture("wave_right")

# Generate motion frames
frames = gesture.generate_motion_frames()

# Save to file
motion = MotionCapture(35, 0.002)  # 35 DOF, 0.002s timestep
motion.frames = frames
motion.save_motion("my_wave.json", {"gesture_name": "wave_right"})
```

#### Playing Back a Motion

```python
from motion_capture import MotionCapture
from motion_replay import MotionPlayer
import mujoco

# Load motion
motion = MotionCapture(35, 0.002)
motion.load_motion("my_wave.json")

# Create player
player = MotionPlayer(35)
player.load_motion(motion)
player.start_playback(loops=3, speed=1.5)  # 3 loops at 1.5x speed

# In simulation loop
while player.is_playing():
    frame = player.step(0.002)
    if frame:
        positions = np.array(frame.joint_positions)
        # Send positions to robot
        mujoco.mj_step(model, data)
```

#### Chaining Multiple Gestures

```python
from motion_replay import MotionSequence

sequence = MotionSequence(35)

# Add gestures in order
sequence.add_motion(wave_motion, loops=1, speed=1.0)
sequence.add_motion(handshake_motion, loops=2, speed=1.0)
sequence.add_motion(heart_motion, loops=1, speed=0.8)

sequence.start()

# In simulation loop
while not sequence.is_finished():
    frame = sequence.step(0.002)
    # Send positions to robot
```

## Data Format

### Motion File (JSON)

Motion files store robot states and control commands:

```json
{
  "metadata": {
    "timestamp": "2026-05-26T00:51:18.469+07:00",
    "num_motors": 35,
    "num_frames": 1500,
    "sample_rate": 0.002,
    "duration": 3.0,
    "gesture_name": "wave_right"
  },
  "frames": [
    {
      "timestamp": 0.0,
      "joint_positions": [0.0, 0.0, ..., 0.0],
      "joint_velocities": [0.0, 0.0, ..., 0.0],
      "motor_kp": [30.0, 30.0, ..., 30.0],
      "motor_kd": [1.0, 1.0, ..., 1.0]
    },
    ...
  ]
}
```

## R1 Motor Layout

R1 has 35 DOF controlled via `unitree_hg` IDL:

| Index | Motor | Type |
|-------|-------|------|
| 0-5 | Left leg | 6 DOF |
| 6-11 | Right leg | 6 DOF |
| 12-13 | Waist | 2 DOF (roll, yaw) |
| 14-19 | Left arm | 6 DOF |
| 20-25 | Right arm | 6 DOF |
| 26-27 | Head | 2 DOF (pitch, yaw) |
| 28-34 | Reserved | 7 DOF |

### Arm DOF (indices 14-19 for left, 20-25 for right)
- [0] shoulder_pitch - Up/down at shoulder
- [1] shoulder_roll - Rotate outward
- [2] shoulder_yaw - Rotate forward/back
- [3] elbow - Bend/extend forearm
- [4] wrist_roll - Twist wrist
- [5] wrist_roll_2 - Secondary wrist twist

## Creating Custom Gestures

Create a new gesture by subclassing `R1Gesture`:

```python
from r1_gestures import R1Gesture
import numpy as np

class MyGesture(R1Gesture):
    def __init__(self):
        super().__init__("my_gesture", duration=2.0)
        self.arm_offset = 20  # Right arm
    
    def get_command_at_time(self, t):
        pos = np.zeros(self.num_motors)
        kp = np.ones(self.num_motors) * 30.0
        kd = np.ones(self.num_motors) * 1.0
        
        # Set leg positions for standing
        pos[0:6] = [0, 0, 0, 0.6, -0.3, 0]  # Left leg
        pos[6:12] = [0, 0, 0, 0.6, -0.3, 0]  # Right leg
        
        # Arm motion (your custom trajectory)
        phase = t / self.duration
        pos[self.arm_offset + 0] = np.sin(phase * 2 * np.pi) * 0.5  # Shoulder pitch
        pos[self.arm_offset + 1] = np.cos(phase * 2 * np.pi) * 0.3  # Shoulder roll
        
        return pos, kp, kd
```

## Advanced Features

### Speed Control
Play motion at different speeds:

```python
player.start_playback(loops=1, speed=2.0)  # 2x speed
player.start_playback(loops=1, speed=0.5)  # Half speed
```

### Looping
Play motion multiple times:

```python
player.start_playback(loops=5)      # 5 times
player.start_playback(loops=0)      # Infinite
```

### Callbacks
Execute code when motion loops complete:

```python
def on_complete():
    print("Motion finished!")

player.on_loop_complete = on_complete
player.start_playback(loops=3)
```

### Interpolation
Get smooth interpolated frames between keyframes:

```python
frame = motion.interpolate_frame(1.234)  # Get state at 1.234 seconds
positions = np.array(frame.joint_positions)
```

## Troubleshooting

### "Motor count mismatch"
- Ensure config.py has `ROBOT = "r1"`
- R1 has 35 DOF - this is fixed

### Gesture looks jerky
- Increase playback speed slightly or use `InterpolatedMotionPlayer`
- Check motor gains (kp, kd values) in gesture definition

### Gesture plays too fast/slow
- Adjust `speed` parameter in `start_playback()`
- Check simulator timestep in config.py

### Can't record motion
- Ensure viewer has keyboard focus
- Check that DDS is properly initialized
- Verify `data/gestures` directory exists

## Performance Tips

1. **Pre-generate gestures** - Generate common gestures once, load many times
2. **Use sequences** - Chain gestures efficiently with `MotionSequence`
3. **Lower speed for smooth playback** - Reduce `speed_factor` if jerky
4. **Interpolate for screen sync** - Use `InterpolatedMotionPlayer` for 60+ fps displays

## References

- R1 Motor Documentation: https://support.unitree.com/
- MuJoCo Documentation: https://mujoco.readthedocs.io/
- SDK2 IDL Reference: https://github.com/unitreerobotics/unitree_sdk2

---

Last updated: May 2026
