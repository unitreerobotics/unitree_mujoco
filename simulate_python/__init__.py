"""
Unitree MuJoCo Simulator - Python Implementation
Provides motion capture and replay functionality for robot control.
"""

from .motion_capture import MotionCapture, MotionFrame
from .motion_replay import MotionPlayer, MotionSequence, InterpolatedMotionPlayer
from .r1_gestures import get_gesture, list_gestures, R1Gesture

__version__ = "1.0.0"
__all__ = [
    "MotionCapture",
    "MotionFrame",
    "MotionPlayer",
    "MotionSequence",
    "InterpolatedMotionPlayer",
    "get_gesture",
    "list_gestures",
    "R1Gesture",
]
