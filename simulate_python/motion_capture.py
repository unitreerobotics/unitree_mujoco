import json
import numpy as np
from dataclasses import dataclass, asdict
from typing import List, Dict, Optional
from datetime import datetime


@dataclass
class MotionFrame:
    """Single frame of captured motion data."""
    timestamp: float
    joint_positions: List[float]
    joint_velocities: List[float]
    motor_kp: List[float]  # Position gains used
    motor_kd: List[float]  # Velocity gains used


class MotionCapture:
    """Captures and records robot motion data to file."""
    
    def __init__(self, num_motors: int, sample_rate: float = 0.002):
        """
        Initialize motion capture.
        
        Args:
            num_motors: Number of motors in the robot
            sample_rate: Simulation timestep (seconds)
        """
        self.num_motors = num_motors
        self.sample_rate = sample_rate
        self.frames: List[MotionFrame] = []
        self.is_recording = False
        self.start_time = None
        
    def start_recording(self):
        """Start recording motion data."""
        self.is_recording = True
        self.frames = []
        self.start_time = 0.0
        print("[Motion Capture] Recording started")
        
    def stop_recording(self) -> bool:
        """Stop recording and return success."""
        if not self.is_recording:
            print("[Motion Capture] Not currently recording")
            return False
        self.is_recording = False
        print(f"[Motion Capture] Recording stopped. Captured {len(self.frames)} frames")
        return len(self.frames) > 0
    
    def capture_frame(self, 
                     timestamp: float,
                     joint_positions: np.ndarray,
                     joint_velocities: np.ndarray,
                     motor_kp: np.ndarray,
                     motor_kd: np.ndarray):
        """Capture a single frame of motion data."""
        if not self.is_recording:
            return
            
        frame = MotionFrame(
            timestamp=timestamp,
            joint_positions=joint_positions.tolist(),
            joint_velocities=joint_velocities.tolist(),
            motor_kp=motor_kp.tolist(),
            motor_kd=motor_kd.tolist()
        )
        self.frames.append(frame)
    
    def save_motion(self, filepath: str, metadata: Optional[Dict] = None) -> bool:
        """
        Save captured motion to JSON file.
        
        Args:
            filepath: Path to save motion file
            metadata: Optional metadata dict (gesture name, description, etc)
        
        Returns:
            True if saved successfully
        """
        if not self.frames:
            print("[Motion Capture] No frames to save")
            return False
        
        data = {
            "metadata": {
                "timestamp": datetime.now().isoformat(),
                "num_motors": self.num_motors,
                "num_frames": len(self.frames),
                "sample_rate": self.sample_rate,
                "duration": self.frames[-1].timestamp if self.frames else 0.0,
                **(metadata or {})
            },
            "frames": [asdict(f) for f in self.frames]
        }
        
        try:
            with open(filepath, 'w') as f:
                json.dump(data, f, indent=2)
            print(f"[Motion Capture] Motion saved to {filepath}")
            return True
        except Exception as e:
            print(f"[Motion Capture] Error saving motion: {e}")
            return False
    
    def load_motion(self, filepath: str) -> bool:
        """
        Load motion from JSON file.
        
        Args:
            filepath: Path to motion file
        
        Returns:
            True if loaded successfully
        """
        try:
            with open(filepath, 'r') as f:
                data = json.load(f)
            
            # Validate metadata
            meta = data.get("metadata", {})
            if meta.get("num_motors") != self.num_motors:
                print(f"[Motion Capture] Motor count mismatch: file has {meta.get('num_motors')}, "
                      f"robot has {self.num_motors}")
                return False
            
            # Load frames
            self.frames = [
                MotionFrame(
                    timestamp=f["timestamp"],
                    joint_positions=f["joint_positions"],
                    joint_velocities=f["joint_velocities"],
                    motor_kp=f["motor_kp"],
                    motor_kd=f["motor_kd"]
                )
                for f in data.get("frames", [])
            ]
            print(f"[Motion Capture] Loaded {len(self.frames)} frames from {filepath}")
            return True
        except Exception as e:
            print(f"[Motion Capture] Error loading motion: {e}")
            return False
    
    def get_frame_at_index(self, index: int) -> Optional[MotionFrame]:
        """Get a specific frame by index."""
        if 0 <= index < len(self.frames):
            return self.frames[index]
        return None
    
    def get_duration(self) -> float:
        """Get total recorded duration in seconds."""
        if not self.frames:
            return 0.0
        return self.frames[-1].timestamp
    
    def interpolate_frame(self, time: float) -> Optional[MotionFrame]:
        """
        Interpolate motion data at a specific time.
        Returns exact frame if time matches, or interpolated values.
        """
        if not self.frames:
            return None
        
        # Find surrounding frames
        for i in range(len(self.frames) - 1):
            if self.frames[i].timestamp <= time <= self.frames[i+1].timestamp:
                f0 = self.frames[i]
                f1 = self.frames[i+1]
                
                # Linear interpolation ratio
                dt = f1.timestamp - f0.timestamp
                if dt < 1e-6:
                    return f0
                alpha = (time - f0.timestamp) / dt
                
                # Interpolate positions
                pos = [
                    f0.joint_positions[j] + alpha * (f1.joint_positions[j] - f0.joint_positions[j])
                    for j in range(len(f0.joint_positions))
                ]
                
                # Interpolate velocities
                vel = [
                    f0.joint_velocities[j] + alpha * (f1.joint_velocities[j] - f0.joint_velocities[j])
                    for j in range(len(f0.joint_velocities))
                ]
                
                # Interpolate gains
                kp = [
                    f0.motor_kp[j] + alpha * (f1.motor_kp[j] - f0.motor_kp[j])
                    for j in range(len(f0.motor_kp))
                ]
                
                kd = [
                    f0.motor_kd[j] + alpha * (f1.motor_kd[j] - f0.motor_kd[j])
                    for j in range(len(f0.motor_kd))
                ]
                
                return MotionFrame(
                    timestamp=time,
                    joint_positions=pos,
                    joint_velocities=vel,
                    motor_kp=kp,
                    motor_kd=kd
                )
        
        # Return last frame if time exceeds duration
        return self.frames[-1] if self.frames else None
    
    def get_info(self) -> str:
        """Get human-readable info about captured motion."""
        if not self.frames:
            return "No motion captured"
        
        duration = self.get_duration()
        return (f"Motion: {len(self.frames)} frames, "
                f"Duration: {duration:.2f}s, "
                f"Motors: {self.num_motors}")
