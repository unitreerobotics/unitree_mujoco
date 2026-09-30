#!/usr/bin/env python3
"""
Quick test script to verify motion capture and replay systems work.
Tests without requiring MuJoCo viewer.
"""

import sys
import os

# Add parent directory to path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

import json
import numpy as np
from motion_capture import MotionCapture
from motion_replay import MotionPlayer, MotionSequence
from r1_gestures import get_gesture, list_gestures


def test_gesture_generation():
    """Test gesture generation."""
    print("\n" + "="*50)
    print("TEST: Gesture Generation")
    print("="*50)
    
    for gesture_name in list_gestures():
        gesture = get_gesture(gesture_name)
        frames = gesture.generate_motion_frames()
        
        assert len(frames) > 0, f"{gesture_name}: No frames generated"
        assert frames[0].timestamp == 0.0, f"{gesture_name}: First frame time != 0"
        assert all(len(f.joint_positions) == 35 for f in frames), f"{gesture_name}: Wrong joint count"
        
        print(f"✓ {gesture_name}: {len(frames)} frames, duration {gesture.duration}s")
    
    print("✓ All gestures generated successfully")


def test_motion_capture():
    """Test motion capture save/load."""
    print("\n" + "="*50)
    print("TEST: Motion Capture Save/Load")
    print("="*50)
    
    # Create a test motion
    motion1 = MotionCapture(35, 0.002)
    gesture = get_gesture("wave_right")
    motion1.frames = gesture.generate_motion_frames()
    
    # Save
    test_file = "/tmp/test_motion.json"
    assert motion1.save_motion(test_file, {"test": "data"}), "Failed to save motion"
    print(f"✓ Saved motion to {test_file}")
    
    # Verify file format
    with open(test_file, 'r') as f:
        data = json.load(f)
    
    assert "metadata" in data, "Missing metadata"
    assert "frames" in data, "Missing frames"
    assert data["metadata"]["num_motors"] == 35, "Wrong motor count in file"
    assert len(data["frames"]) > 0, "No frames in file"
    print("✓ File format verified")
    
    # Load
    motion2 = MotionCapture(35, 0.002)
    assert motion2.load_motion(test_file), "Failed to load motion"
    assert len(motion2.frames) == len(motion1.frames), "Frame count mismatch"
    print(f"✓ Loaded motion ({len(motion2.frames)} frames)")
    
    # Cleanup
    os.remove(test_file)


def test_motion_player():
    """Test motion player."""
    print("\n" + "="*50)
    print("TEST: Motion Player")
    print("="*50)
    
    # Create motion
    motion = MotionCapture(35, 0.002)
    gesture = get_gesture("handshake_right")
    motion.frames = gesture.generate_motion_frames()
    
    # Create player
    player = MotionPlayer(35)
    assert player.load_motion(motion), "Failed to load motion"
    print(f"✓ Loaded motion: {player.motion.get_info()}")
    
    # Start playback
    assert player.start_playback(loops=2, speed=1.5), "Failed to start playback"
    assert player.is_playing, "Player not playing after start"
    print("✓ Playback started (2 loops, 1.5x speed)")
    
    # Simulate playback
    dt = 0.002
    frame_count = 0
    last_time = 0.0
    
    while not player.is_finished():
        frame = player.step(dt)
        if frame:
            frame_count += 1
            last_time = frame.timestamp
    
    assert frame_count > 0, "No frames played"
    print(f"✓ Played {frame_count} frames, last time: {last_time:.2f}s")


def test_motion_sequence():
    """Test motion sequence."""
    print("\n" + "="*50)
    print("TEST: Motion Sequence")
    print("="*50)
    
    # Create motions
    sequence = MotionSequence(35)
    
    motion1 = MotionCapture(35, 0.002)
    gesture1 = get_gesture("wave_left")
    motion1.frames = gesture1.generate_motion_frames()
    
    motion2 = MotionCapture(35, 0.002)
    gesture2 = get_gesture("heart")
    motion2.frames = gesture2.generate_motion_frames()
    
    # Add to sequence
    sequence.add_motion(motion1, loops=1, speed=1.0)
    sequence.add_motion(motion2, loops=1, speed=1.2)
    
    print(f"✓ Created sequence with {len(sequence.sequence)} motions")
    
    # Play sequence
    assert sequence.start(), "Failed to start sequence"
    print("✓ Sequence started")
    
    # Simulate playback
    dt = 0.002
    frame_count = 0
    
    while not sequence.is_finished():
        frame = sequence.step(dt)
        if frame:
            frame_count += 1
    
    assert frame_count > 0, "No frames played in sequence"
    print(f"✓ Played sequence ({frame_count} frames total)")


def test_interpolation():
    """Test frame interpolation."""
    print("\n" + "="*50)
    print("TEST: Frame Interpolation")
    print("="*50)
    
    # Create motion with specific frames
    motion = MotionCapture(35, 0.002)
    gesture = get_gesture("wave_right")
    motion.frames = gesture.generate_motion_frames()
    
    # Test interpolation at various times
    test_times = [0.0, 0.5, 1.0, 1.5]
    
    for t in test_times:
        if t <= motion.get_duration():
            frame = motion.interpolate_frame(t)
            assert frame is not None, f"No frame at t={t}"
            assert len(frame.joint_positions) == 35, "Wrong joint count"
            print(f"✓ Interpolated at t={t:.2f}s")


def test_motor_count_mismatch():
    """Test error handling for motor count mismatch."""
    print("\n" + "="*50)
    print("TEST: Motor Count Mismatch Detection")
    print("="*50)
    
    # Create motion with 35 DOF
    motion = MotionCapture(35, 0.002)
    gesture = get_gesture("wave_right")
    motion.frames = gesture.generate_motion_frames()
    
    # Try to load with wrong motor count
    player = MotionPlayer(20)  # Wrong count
    
    result = player.load_motion(motion)
    assert result == False, "Should reject motor count mismatch"
    print("✓ Motor count mismatch correctly detected")


def main():
    """Run all tests."""
    print("\n" + "="*60)
    print("R1 GESTURE SYSTEM TEST SUITE")
    print("="*60)
    
    try:
        test_gesture_generation()
        test_motion_capture()
        test_motion_player()
        test_motion_sequence()
        test_interpolation()
        test_motor_count_mismatch()
        
        print("\n" + "="*60)
        print("✓✓✓ ALL TESTS PASSED ✓✓✓")
        print("="*60)
        return 0
    
    except AssertionError as e:
        print(f"\n✗ TEST FAILED: {e}")
        return 1
    except Exception as e:
        print(f"\n✗ UNEXPECTED ERROR: {e}")
        import traceback
        traceback.print_exc()
        return 1


if __name__ == "__main__":
    sys.exit(main())
