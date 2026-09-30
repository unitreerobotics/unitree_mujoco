"""
R1 Gesture definitions - pre-scripted motions for common gestures.

R1 Motor Layout (35 DOF):
- Legs: 12 DOF (hip_pitch, hip_roll, hip_yaw, knee, ankle_pitch, ankle_roll x2)
- Waist: 2 DOF (roll, yaw)
- Arms: 10 DOF (shoulder_pitch, shoulder_roll, shoulder_yaw, elbow, wrist_roll x2)
- Head: 2 DOF (pitch, yaw)

Motor indices mapping for unitree_hg (R1):
0-11:  Left leg (6) + Right leg (6)
12-13: Waist (roll, yaw)
14-19: Left arm (shoulder_pitch, shoulder_roll, shoulder_yaw, elbow, wrist_roll x2)
20-25: Right arm (shoulder_pitch, shoulder_roll, shoulder_yaw, elbow, wrist_roll x2)
26-27: Head (pitch, yaw)
28-34: Additional DOF (reserved/extension)
"""

import numpy as np
from typing import List, Tuple
from motion_capture import MotionFrame


class R1Gesture:
    """Base class for R1 gestures."""
    
    def __init__(self, name: str, duration: float, sample_rate: float = 0.002):
        self.name = name
        self.duration = duration
        self.sample_rate = sample_rate
        self.num_motors = 35
        
    def get_frame_count(self) -> int:
        """Calculate number of frames for this gesture."""
        return int(self.duration / self.sample_rate)
    
    def get_command_at_time(self, t: float) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        """
        Get motor command at time t.
        
        Returns:
            (positions, kp_gains, kd_gains) tuples as numpy arrays
        """
        raise NotImplementedError
    
    def generate_motion_frames(self) -> List[MotionFrame]:
        """Generate all motion frames for this gesture."""
        frames = []
        t = 0.0
        
        while t <= self.duration:
            pos, kp, kd = self.get_command_at_time(t)
            
            frame = MotionFrame(
                timestamp=t,
                joint_positions=pos.tolist(),
                joint_velocities=[0.0] * self.num_motors,  # Scripted motions use position control
                motor_kp=kp.tolist(),
                motor_kd=kd.tolist()
            )
            frames.append(frame)
            t += self.sample_rate
        
        return frames


class WaveGesture(R1Gesture):
    """Wave hand gesture - continuous waving motion."""
    
    def __init__(self, duration: float = 3.0, hand: str = "right"):
        """
        Args:
            duration: Total gesture duration (seconds)
            hand: "left" or "right" hand to wave
        """
        super().__init__(f"wave_{hand}", duration)
        self.hand = hand
        self.arm_offset = 20 if hand == "right" else 14  # Right arm starts at index 20
        
    def get_command_at_time(self, t: float) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        """Wave hand by rotating shoulder and elbow."""
        pos = np.zeros(self.num_motors)
        kp = np.ones(self.num_motors) * 30.0
        kd = np.ones(self.num_motors) * 1.0
        
        # Keep robot standing (basic leg position)
        self._set_standing_pose(pos)
        
        # Wave motion: oscillate shoulder and elbow
        phase = (t / self.duration) * 2 * np.pi  # Full cycles during gesture
        
        # Shoulder yaw: wide oscillation (move arm side to side)
        pos[self.arm_offset + 2] = 0.8 * np.sin(phase)  # shoulder_yaw
        
        # Elbow: small oscillation (bend/straighten)
        pos[self.arm_offset + 3] = 1.2 + 0.3 * np.sin(phase * 2)  # elbow
        
        return pos, kp, kd
    
    def _set_standing_pose(self, pos: np.ndarray):
        """Set legs to stable standing position."""
        # Simple standing pose for legs (keep center)
        pos[0] = 0.0   # left_hip_pitch
        pos[1] = 0.0   # left_hip_roll
        pos[2] = 0.0   # left_hip_yaw
        pos[3] = 0.6   # left_knee
        pos[4] = -0.3  # left_ankle_pitch
        pos[5] = 0.0   # left_ankle_roll
        
        pos[6] = 0.0   # right_hip_pitch
        pos[7] = 0.0   # right_hip_roll
        pos[8] = 0.0   # right_hip_yaw
        pos[9] = 0.6   # right_knee
        pos[10] = -0.3  # right_ankle_pitch
        pos[11] = 0.0   # right_ankle_roll
        
        # Waist neutral
        pos[12] = 0.0  # waist_roll
        pos[13] = 0.0  # waist_yaw
        
        # Other arm neutral
        other_offset = 20 if self.hand == "left" else 14
        pos[other_offset + 0] = 0.0  # shoulder_pitch
        pos[other_offset + 1] = 1.0  # shoulder_roll
        pos[other_offset + 2] = 0.0  # shoulder_yaw
        pos[other_offset + 3] = 1.5  # elbow
        pos[other_offset + 4] = 0.0  # wrist_roll
        pos[other_offset + 5] = 0.0  # wrist_roll_2
        
        # Head neutral
        pos[26] = 0.0  # head_pitch
        pos[27] = 0.0  # head_yaw


class HandshakeGesture(R1Gesture):
    """Handshake gesture - hand forward and down motion."""
    
    def __init__(self, duration: float = 2.0, hand: str = "right"):
        """
        Args:
            duration: Total gesture duration (seconds)
            hand: "left" or "right" hand
        """
        super().__init__(f"handshake_{hand}", duration)
        self.hand = hand
        self.arm_offset = 20 if hand == "right" else 14
        
    def get_command_at_time(self, t: float) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        """Extend arm forward and shake vertically."""
        pos = np.zeros(self.num_motors)
        kp = np.ones(self.num_motors) * 35.0
        kd = np.ones(self.num_motors) * 1.5
        
        self._set_standing_pose(pos)
        
        # Handshake phases:
        # 0-0.5: Raise arm forward
        # 0.5-2.0: Shake up and down
        
        if t < 0.5:
            # Raise arm forward
            alpha = t / 0.5
            pos[self.arm_offset + 0] = alpha * 0.5   # shoulder_pitch forward
            pos[self.arm_offset + 1] = alpha * -0.3  # shoulder_roll inward
            pos[self.arm_offset + 3] = 0.5 + alpha * 1.0  # extend elbow
        else:
            # Maintain position and shake
            pos[self.arm_offset + 0] = 0.5   # shoulder_pitch
            pos[self.arm_offset + 1] = -0.3  # shoulder_roll
            pos[self.arm_offset + 3] = 1.5   # elbow extended
            
            # Shake: small up-down motion at waist
            shake_time = t - 0.5
            shake_phase = (shake_time / 1.5) * 4 * np.pi  # 2 shakes
            pos[12] += 0.15 * np.sin(shake_phase)  # waist_roll shake
        
        return pos, kp, kd
    
    def _set_standing_pose(self, pos: np.ndarray):
        """Set legs to stable standing position."""
        pos[0] = 0.0   # left_hip_pitch
        pos[1] = 0.0   # left_hip_roll
        pos[2] = 0.0   # left_hip_yaw
        pos[3] = 0.6   # left_knee
        pos[4] = -0.3  # left_ankle_pitch
        pos[5] = 0.0   # left_ankle_roll
        
        pos[6] = 0.0   # right_hip_pitch
        pos[7] = 0.0   # right_hip_roll
        pos[8] = 0.0   # right_hip_yaw
        pos[9] = 0.6   # right_knee
        pos[10] = -0.3  # right_ankle_pitch
        pos[11] = 0.0   # right_ankle_roll
        
        pos[12] = 0.0  # waist_roll
        pos[13] = 0.0  # waist_yaw
        
        # Other arm neutral
        other_offset = 20 if self.hand == "left" else 14
        pos[other_offset + 0] = 0.0
        pos[other_offset + 1] = 1.0
        pos[other_offset + 2] = 0.0
        pos[other_offset + 3] = 1.5
        pos[other_offset + 4] = 0.0
        pos[other_offset + 5] = 0.0
        
        pos[26] = 0.0  # head_pitch
        pos[27] = 0.0  # head_yaw


class HeartGesture(R1Gesture):
    """Heart shape gesture - both hands forming heart shape."""
    
    def __init__(self, duration: float = 3.0):
        super().__init__("heart", duration)
        
    def get_command_at_time(self, t: float) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        """Form heart shape with both hands."""
        pos = np.zeros(self.num_motors)
        kp = np.ones(self.num_motors) * 32.0
        kd = np.ones(self.num_motors) * 1.2
        
        self._set_standing_pose(pos)
        
        # Heart formation phases:
        # 0-1.0: Move arms to form heart
        # 1.0-3.0: Hold heart shape with slight sway
        
        if t < 1.0:
            alpha = t / 1.0
            # Both arms up and inward to form heart
            self._form_heart_pose(pos, alpha)
        else:
            # Hold heart with gentle sway
            sway = 0.1 * np.sin((t - 1.0) * 2)
            self._form_heart_pose(pos, 1.0)
            pos[12] += sway  # Gentle waist sway
        
        return pos, kp, kd
    
    def _form_heart_pose(self, pos: np.ndarray, progress: float):
        """Set arm positions to form heart shape."""
        # Left arm
        left_offset = 14
        pos[left_offset + 0] = progress * 1.2   # shoulder_pitch up
        pos[left_offset + 1] = progress * 0.8   # shoulder_roll right
        pos[left_offset + 2] = progress * 0.5   # shoulder_yaw in
        pos[left_offset + 3] = 0.3 + progress * 0.7  # elbow bent
        
        # Right arm (mirror of left)
        right_offset = 20
        pos[right_offset + 0] = progress * 1.2   # shoulder_pitch up
        pos[right_offset + 1] = progress * -0.8  # shoulder_roll left
        pos[right_offset + 2] = progress * -0.5  # shoulder_yaw in
        pos[right_offset + 3] = 0.3 + progress * 0.7  # elbow bent
    
    def _set_standing_pose(self, pos: np.ndarray):
        """Set legs to stable standing position."""
        pos[0] = 0.0   # left_hip_pitch
        pos[1] = 0.0   # left_hip_roll
        pos[2] = 0.0   # left_hip_yaw
        pos[3] = 0.6   # left_knee
        pos[4] = -0.3  # left_ankle_pitch
        pos[5] = 0.0   # left_ankle_roll
        
        pos[6] = 0.0   # right_hip_pitch
        pos[7] = 0.0   # right_hip_roll
        pos[8] = 0.0   # right_hip_yaw
        pos[9] = 0.6   # right_knee
        pos[10] = -0.3  # right_ankle_pitch
        pos[11] = 0.0   # right_ankle_roll
        
        pos[12] = 0.0  # waist_roll
        pos[13] = 0.0  # waist_yaw
        
        pos[26] = 0.0  # head_pitch
        pos[27] = 0.0  # head_yaw


# Gesture factory
GESTURES = {
    "wave_right": lambda: WaveGesture(hand="right"),
    "wave_left": lambda: WaveGesture(hand="left"),
    "handshake_right": lambda: HandshakeGesture(hand="right"),
    "handshake_left": lambda: HandshakeGesture(hand="left"),
    "heart": lambda: HeartGesture(),
}


def get_gesture(name: str) -> R1Gesture:
    """Get a gesture by name."""
    if name not in GESTURES:
        available = list(GESTURES.keys())
        raise ValueError(f"Unknown gesture '{name}'. Available: {available}")
    return GESTURES[name]()


def list_gestures() -> List[str]:
    """List all available gestures."""
    return list(GESTURES.keys())
