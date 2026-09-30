#!/usr/bin/env python3
"""
R1 Gesture Recording and Replay Example

This example demonstrates:
1. Loading the R1 robot in MuJoCo
2. Recording/generating gestures (wave, handshake, heart)
3. Replaying recorded motions
4. Chaining multiple gestures in sequence

Usage:
    python3 example_r1_gestures.py [record|replay|generate]
    
Commands:
    generate    - Generate all gesture files (wave, handshake, heart)
    replay      - Replay all generated gestures in sequence
    record      - Interactive recording mode (press 'r' to start/stop, 'q' to quit)
"""

import sys
import os
import time
import numpy as np
import mujoco
import mujoco_viewer

# Add simulate_python to path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'simulate_python'))

from unitree_sdk2py.core.channel import ChannelFactoryInitialize
from unitree_sdk2py.utils.crc import CRC
import config
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_
from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowCmd_

from motion_capture import MotionCapture
from motion_replay import MotionPlayer, MotionSequence
from r1_gestures import get_gesture, list_gestures


class R1GestureController:
    """Controls R1 robot gestures with motion capture and replay."""
    
    def __init__(self):
        # Load MuJoCo model
        xml_file = config.ROBOT_SCENE
        self.model = mujoco.MjModel.from_xml_path(xml_file)
        self.data = mujoco.MjData(self.model)
        self.viewer = None
        
        # Get number of motors
        self.num_motors = self.model.nu
        self.dt = self.model.opt.timestep
        
        print(f"[R1Controller] Loaded {config.ROBOT} with {self.num_motors} DOF")
        print(f"[R1Controller] Timestep: {self.dt}s")
        
        # Motion capture and replay
        self.motion_capture = MotionCapture(self.num_motors, self.dt)
        self.motion_player = MotionPlayer(self.num_motors)
        self.motion_sequence = MotionSequence(self.num_motors)
        
        # DDS setup
        self.crc = CRC()
        self._init_dds()
        
        # UI state
        self.recording = False
        self.replaying = False
        
    def _init_dds(self):
        """Initialize DDS communication."""
        try:
            ChannelFactoryInitialize(1, "lo")  # Use loopback for simulation
            print("[R1Controller] DDS initialized on loopback")
        except Exception as e:
            print(f"[R1Controller] Warning: DDS init failed: {e}")
    
    def _get_current_joint_state(self) -> tuple:
        """Get current joint positions and velocities from simulator."""
        # Get actuator DOF indices
        actuator_indices = []
        for i in range(self.model.nu):
            actuator_indices.append(self.model.actuator_trnid[i*2])
        
        # Extract joint values
        pos = np.array([self.data.qpos[i+7] for i in actuator_indices])  # Skip floating base
        vel = np.array([self.data.qvel[i+6] for i in actuator_indices])  # Skip floating base velocity
        
        return pos, vel
    
    def _send_motor_command(self, positions: np.ndarray, kp: np.ndarray, kd: np.ndarray, tau: np.ndarray = None):
        """Send motor command to simulator."""
        if tau is None:
            tau = np.zeros(self.num_motors)
        
        for i in range(self.num_motors):
            self.data.ctrl[i] = positions[i]
    
    def setup_viewer(self):
        """Setup MuJoCo viewer."""
        if self.viewer is None:
            self.viewer = mujoco_viewer.MujocoViewer(self.model, self.data)
    
    def close_viewer(self):
        """Close viewer."""
        if self.viewer:
            self.viewer.close()
            self.viewer = None
    
    def generate_gestures(self):
        """Generate and save all gesture files."""
        print("\n" + "="*50)
        print("GENERATING GESTURES")
        print("="*50)
        
        gestures_dir = os.path.join(os.path.dirname(__file__), '..', 'data', 'gestures')
        os.makedirs(gestures_dir, exist_ok=True)
        
        for gesture_name in list_gestures():
            print(f"\nGenerating: {gesture_name}")
            gesture = get_gesture(gesture_name)
            
            # Generate motion frames
            frames = gesture.generate_motion_frames()
            print(f"  Generated {len(frames)} frames ({gesture.duration}s)")
            
            # Create motion capture and load frames
            motion = MotionCapture(self.num_motors, self.dt)
            motion.frames = frames
            
            # Save motion
            filepath = os.path.join(gestures_dir, f"{gesture_name}.json")
            if motion.save_motion(filepath, {"gesture_name": gesture_name}):
                print(f"  Saved to: {filepath}")
        
        print("\n✓ All gestures generated successfully!")
        return gestures_dir
    
    def replay_gestures(self, gestures_dir: str = None):
        """Replay all gestures in sequence."""
        if gestures_dir is None:
            gestures_dir = os.path.join(os.path.dirname(__file__), '..', 'data', 'gestures')
        
        if not os.path.exists(gestures_dir):
            print(f"Gestures directory not found: {gestures_dir}")
            print("Run 'python3 example_r1_gestures.py generate' first")
            return
        
        print("\n" + "="*50)
        print("REPLAYING GESTURES")
        print("="*50)
        
        self.setup_viewer()
        
        # Load all gestures
        gesture_files = sorted([f for f in os.listdir(gestures_dir) if f.endswith('.json')])
        
        if not gesture_files:
            print(f"No gesture files found in {gestures_dir}")
            return
        
        print(f"Found {len(gesture_files)} gestures:")
        for f in gesture_files:
            print(f"  - {f}")
        
        # Build sequence
        sequence = MotionSequence(self.num_motors)
        
        for gesture_file in gesture_files:
            filepath = os.path.join(gestures_dir, gesture_file)
            motion = MotionCapture(self.num_motors, self.dt)
            
            if motion.load_motion(filepath):
                # Replay each gesture 2 times
                sequence.add_motion(motion, loops=2, speed=1.0)
        
        # Start sequence playback
        if not sequence.start():
            print("Failed to start sequence")
            return
        
        # Simulation loop
        print("\nStarting playback (press 'q' in viewer to quit)...")
        time.sleep(1)
        
        running_time = 0.0
        
        try:
            while self.viewer.is_running():
                # Advance sequence
                frame = sequence.step(self.dt)
                
                if frame is None:
                    # Sequence finished
                    print("\n✓ Sequence playback completed!")
                    break
                
                # Apply motor commands from motion
                positions = np.array(frame.joint_positions)
                kp = np.array(frame.motor_kp)
                kd = np.array(frame.motor_kd)
                
                self._send_motor_command(positions, kp, kd)
                
                # Step simulation
                mujoco.mj_step(self.model, self.data)
                
                # Sync viewer
                self.viewer.sync()
                
                running_time += self.dt
                
                if running_time % 1.0 < self.dt:
                    print(f"  {sequence.get_status()}")
        
        except KeyboardInterrupt:
            print("\n✓ Playback interrupted by user")
        
        finally:
            self.close_viewer()
    
    def interactive_record(self):
        """Interactive gesture recording mode."""
        print("\n" + "="*50)
        print("INTERACTIVE RECORDING MODE")
        print("="*50)
        print("Controls:")
        print("  'r' - Start/stop recording")
        print("  's' - Save recording")
        print("  'c' - Clear current recording")
        print("  'q' - Quit")
        print("\nUse the keyboard to control the robot or drag in the viewer.")
        
        self.setup_viewer()
        
        try:
            while self.viewer.is_running():
                # Get keyboard input
                key = self.viewer.get_lastkey()
                
                if key == ord('r'):
                    if self.motion_capture.is_recording:
                        self.motion_capture.stop_recording()
                        self.recording = False
                    else:
                        self.motion_capture.start_recording()
                        self.recording = True
                    time.sleep(0.2)
                
                elif key == ord('s'):
                    if self.motion_capture.frames:
                        gestures_dir = os.path.join(os.path.dirname(__file__), '..', 'data', 'gestures')
                        os.makedirs(gestures_dir, exist_ok=True)
                        timestamp = time.strftime("%Y%m%d_%H%M%S")
                        filepath = os.path.join(gestures_dir, f"recorded_{timestamp}.json")
                        self.motion_capture.save_motion(filepath, {"source": "interactive_recording"})
                    else:
                        print("No motion to save")
                    time.sleep(0.2)
                
                elif key == ord('c'):
                    self.motion_capture = MotionCapture(self.num_motors, self.dt)
                    self.recording = False
                    print("Recording cleared")
                    time.sleep(0.2)
                
                elif key == ord('q'):
                    break
                
                # Record current state if recording
                if self.recording:
                    pos, vel = self._get_current_joint_state()
                    kp = np.ones(self.num_motors) * 30.0
                    kd = np.ones(self.num_motors) * 1.0
                    self.motion_capture.capture_frame(
                        self.data.time,
                        pos, vel, kp, kd
                    )
                
                # Step simulation
                mujoco.mj_step(self.model, self.data)
                self.viewer.sync()
        
        except KeyboardInterrupt:
            pass
        finally:
            self.close_viewer()


def main():
    """Main entry point."""
    if len(sys.argv) < 2:
        print(__doc__)
        sys.exit(1)
    
    cmd = sys.argv[1].lower()
    
    controller = R1GestureController()
    
    if cmd == "generate":
        controller.generate_gestures()
    
    elif cmd == "replay":
        controller.replay_gestures()
    
    elif cmd == "record":
        controller.interactive_record()
    
    else:
        print(f"Unknown command: {cmd}")
        print(__doc__)
        sys.exit(1)


if __name__ == "__main__":
    main()
