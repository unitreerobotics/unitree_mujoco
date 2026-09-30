"""
Motion replay system - plays back captured or scripted motions.
"""

import numpy as np
from typing import Optional, List, Callable
from motion_capture import MotionCapture, MotionFrame


class MotionPlayer:
    """Plays back recorded or scripted motions."""
    
    def __init__(self, num_motors: int):
        """
        Initialize motion player.
        
        Args:
            num_motors: Number of motors in the robot
        """
        self.num_motors = num_motors
        self.motion = None
        self.current_time = 0.0
        self.is_playing = False
        self.loop_count = 0
        self.remaining_loops = 0
        self.speed_factor = 1.0
        self.on_loop_complete: Optional[Callable] = None
        
    def load_motion(self, motion: MotionCapture) -> bool:
        """
        Load a motion from MotionCapture object.
        
        Args:
            motion: MotionCapture object with recorded frames
        
        Returns:
            True if loaded successfully
        """
        if not motion.frames:
            print("[MotionPlayer] Motion has no frames")
            return False
        
        if motion.num_motors != self.num_motors:
            print(f"[MotionPlayer] Motor count mismatch: motion has {motion.num_motors}, "
                  f"player expects {self.num_motors}")
            return False
        
        self.motion = motion
        self.current_time = 0.0
        self.is_playing = False
        print(f"[MotionPlayer] Motion loaded: {motion.get_info()}")
        return True
    
    def start_playback(self, loops: int = 1, speed: float = 1.0) -> bool:
        """
        Start playing motion.
        
        Args:
            loops: Number of times to loop the motion (0 = infinite)
            speed: Playback speed factor (1.0 = normal, 2.0 = 2x speed, 0.5 = half speed)
        
        Returns:
            True if playback started successfully
        """
        if self.motion is None:
            print("[MotionPlayer] No motion loaded")
            return False
        
        if speed <= 0:
            print("[MotionPlayer] Speed must be positive")
            return False
        
        self.current_time = 0.0
        self.is_playing = True
        self.loop_count = loops
        self.remaining_loops = loops if loops > 0 else float('inf')
        self.speed_factor = speed
        print(f"[MotionPlayer] Playback started: {loops} loops, {speed}x speed")
        return True
    
    def stop_playback(self):
        """Stop playback immediately."""
        self.is_playing = False
        self.current_time = 0.0
        print("[MotionPlayer] Playback stopped")
    
    def pause_playback(self):
        """Pause playback (can resume)."""
        self.is_playing = False
    
    def resume_playback(self):
        """Resume paused playback."""
        if self.motion is not None:
            self.is_playing = True
    
    def step(self, dt: float) -> Optional[MotionFrame]:
        """
        Advance playback by one timestep.
        
        Args:
            dt: Timestep in seconds
        
        Returns:
            Current motion frame if playing, None if stopped
        """
        if not self.is_playing or self.motion is None:
            return None
        
        # Advance time with speed factor
        self.current_time += dt * self.speed_factor
        
        # Get motion duration
        motion_duration = self.motion.get_duration()
        
        # Check for loop completion
        if self.current_time >= motion_duration:
            self.current_time = 0.0
            
            if self.remaining_loops != float('inf'):
                self.remaining_loops -= 1
            
            if self.remaining_loops <= 0 and self.loop_count > 0:
                self.is_playing = False
                if self.on_loop_complete:
                    self.on_loop_complete()
                print("[MotionPlayer] Playback completed")
                return None
        
        # Get interpolated frame
        return self.motion.interpolate_frame(self.current_time)
    
    def get_status(self) -> str:
        """Get human-readable playback status."""
        if not self.is_playing:
            status = "Stopped"
        else:
            status = "Playing"
        
        if self.motion:
            progress = (self.current_time / self.motion.get_duration() * 100) if self.motion.get_duration() > 0 else 0
            loops_str = f"{int(self.loop_count)} loops" if self.loop_count > 0 else "infinite loops"
            return f"{status}: {progress:.1f}% ({loops_str}, {self.speed_factor}x speed)"
        else:
            return f"{status}: No motion loaded"
    
    def is_finished(self) -> bool:
        """Check if playback is finished."""
        return not self.is_playing


class MotionSequence:
    """Plays multiple motions in sequence."""
    
    def __init__(self, num_motors: int):
        """
        Initialize motion sequence.
        
        Args:
            num_motors: Number of motors in the robot
        """
        self.num_motors = num_motors
        self.player = MotionPlayer(num_motors)
        self.sequence: List[tuple] = []  # List of (motion, loops, speed)
        self.current_index = 0
        self.is_running = False
    
    def add_motion(self, motion: MotionCapture, loops: int = 1, speed: float = 1.0):
        """Add a motion to the sequence."""
        if motion.num_motors != self.num_motors:
            print(f"[MotionSequence] Motor count mismatch in motion {len(self.sequence)}")
            return
        self.sequence.append((motion, loops, speed))
    
    def start(self) -> bool:
        """Start sequence playback."""
        if not self.sequence:
            print("[MotionSequence] Sequence is empty")
            return False
        
        self.current_index = 0
        self.is_running = True
        motion, loops, speed = self.sequence[0]
        self.player.load_motion(motion)
        self.player.on_loop_complete = self._on_motion_complete
        self.player.start_playback(loops, speed)
        print(f"[MotionSequence] Sequence started ({len(self.sequence)} motions)")
        return True
    
    def step(self, dt: float) -> Optional[MotionFrame]:
        """Advance sequence by one timestep."""
        frame = self.player.step(dt)
        
        # If current motion finished, try next
        if self.player.is_finished() and self.current_index < len(self.sequence):
            self._start_next_motion()
        
        return frame
    
    def _on_motion_complete(self):
        """Called when current motion loop completes."""
        self._start_next_motion()
    
    def _start_next_motion(self):
        """Start the next motion in sequence."""
        if self.current_index + 1 < len(self.sequence):
            self.current_index += 1
            motion, loops, speed = self.sequence[self.current_index]
            self.player.load_motion(motion)
            self.player.start_playback(loops, speed)
            print(f"[MotionSequence] Started motion {self.current_index + 1}/{len(self.sequence)}")
        else:
            self.is_running = False
            print("[MotionSequence] Sequence completed")
    
    def is_finished(self) -> bool:
        """Check if sequence is finished."""
        return not self.is_running
    
    def get_status(self) -> str:
        """Get sequence status."""
        if not self.sequence:
            return "Empty sequence"
        idx_str = f"{self.current_index + 1}/{len(self.sequence)}"
        return f"Sequence: Motion {idx_str} - {self.player.get_status()}"


class InterpolatedMotionPlayer:
    """
    Plays motion with smooth interpolation between frames.
    Useful for smoother visual output at different screen refresh rates.
    """
    
    def __init__(self, num_motors: int):
        self.num_motors = num_motors
        self.player = MotionPlayer(num_motors)
        self.last_frame: Optional[MotionFrame] = None
    
    def load_motion(self, motion: MotionCapture) -> bool:
        return self.player.load_motion(motion)
    
    def start_playback(self, loops: int = 1, speed: float = 1.0) -> bool:
        self.last_frame = None
        return self.player.start_playback(loops, speed)
    
    def stop_playback(self):
        self.player.stop_playback()
    
    def step(self, dt: float) -> Optional[MotionFrame]:
        """Get next frame with interpolation."""
        frame = self.player.step(dt)
        self.last_frame = frame
        return frame
    
    def get_joint_positions(self, time_offset: float = 0.0) -> Optional[np.ndarray]:
        """Get interpolated joint positions at current or offset time."""
        if self.last_frame:
            return np.array(self.last_frame.joint_positions)
        return None
    
    def get_joint_velocities(self, time_offset: float = 0.0) -> Optional[np.ndarray]:
        """Get joint velocities."""
        if self.last_frame:
            return np.array(self.last_frame.joint_velocities)
        return None
    
    def get_motor_gains(self) -> Optional[tuple]:
        """Get motor (kp, kd) gains."""
        if self.last_frame:
            return (np.array(self.last_frame.motor_kp), 
                   np.array(self.last_frame.motor_kd))
        return None
    
    def is_playing(self) -> bool:
        return self.player.is_playing
    
    def is_finished(self) -> bool:
        return self.player.is_finished()
