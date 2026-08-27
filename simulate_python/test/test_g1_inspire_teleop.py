"""DDS integration test for the G1 body and Inspire hands.

Run g1_inspire_sim.py first, then run this in another terminal:

    python test/test_g1_inspire_teleop.py
"""

import sys
import time

import numpy as np

from unitree_sdk2py.core.channel import (
    ChannelFactoryInitialize,
    ChannelPublisher,
    ChannelSubscriber,
)
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_ as HGLowCmd_
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_ as HGLowState_
from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowCmd_
from unitree_sdk2py.idl.unitree_go.msg.dds_ import MotorCmds_, MotorStates_
from unitree_sdk2py.idl.default import unitree_go_msg_dds__MotorCmd_
from unitree_sdk2py.utils.crc import CRC

DOMAIN_ID = 1
KP, KD = 80.0, 3.0

# G1 29-DoF DDS indices
LEFT_SHOULDER_PITCH = 15
state = {"lowstate": None, "inspire": None}


def wait_for(topic, timeout=5.0):
    deadline = time.monotonic() + timeout
    while state[topic] is None and time.monotonic() < deadline:
        time.sleep(0.05)
    if state[topic] is None:
        raise AssertionError(f"no rt/{topic} received within {timeout}s")


def hand_state():
    msg = state["inspire"]
    assert len(msg.states) == 12, f"expected 12 hand states, got {len(msg.states)}"
    return np.array([motor.q for motor in msg.states])


def main():
    ChannelFactoryInitialize(DOMAIN_ID, sys.argv[1] if len(sys.argv) > 1 else "lo")

    lowcmd_pub = ChannelPublisher("rt/lowcmd", HGLowCmd_)
    lowcmd_pub.Init()
    inspire_pub = ChannelPublisher("rt/inspire/cmd", MotorCmds_)
    inspire_pub.Init()

    lowstate_sub = ChannelSubscriber("rt/lowstate", HGLowState_)
    lowstate_sub.Init(lambda msg: state.update(lowstate=msg), 10)
    inspire_sub = ChannelSubscriber("rt/inspire/state", MotorStates_)
    inspire_sub.Init(lambda msg: state.update(inspire=msg), 10)

    crc = CRC()
    cmd = unitree_hg_msg_dds__LowCmd_()
    hand_cmd = MotorCmds_()
    hand_cmd.cmds = [unitree_go_msg_dds__MotorCmd_() for _ in range(12)]

    wait_for("lowstate")
    wait_for("inspire")
    ls = state["lowstate"]
    assert len(ls.motor_state) >= 29, "lowstate is missing G1 body motors"
    assert len(hand_cmd.cmds) == 12
    for i in range(29):
        cmd.motor_cmd[i].q = ls.motor_state[i].q
        cmd.motor_cmd[i].kp = KP
        cmd.motor_cmd[i].kd = KD

    def drive(seconds, hand_q, shoulder_q=None):
        values = np.broadcast_to(hand_q, (12,))
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            if shoulder_q is not None:
                cmd.motor_cmd[LEFT_SHOULDER_PITCH].q = shoulder_q
            cmd.crc = crc.Crc(cmd)
            lowcmd_pub.Write(cmd)
            for i, value in enumerate(values):
                hand_cmd.cmds[i].q = float(value)
            inspire_pub.Write(hand_cmd)
            time.sleep(0.01)

    # No hand command should change the model's startup pose.
    startup = hand_state()
    time.sleep(0.5)
    np.testing.assert_allclose(hand_state(), startup, atol=0.02)

    drive(4.0, 0.0)
    closed = hand_state()
    assert np.max(closed) < 0.23, f"hands did not close: {closed}"
    assert np.max(np.abs(closed - startup)) > 0.5, "hand command caused no motion"

    drive(5.0, 1.0)
    opened = hand_state()
    np.testing.assert_allclose(opened, 1.0, atol=0.13)

    drive(3.0, 0.5)
    midpoint = hand_state()
    np.testing.assert_allclose(midpoint, 0.5, atol=0.12)

    # DDS index 0 is right pinky; changing it must not move the other fingers.
    individual = np.ones(12)
    individual[0] = 0.0
    drive(4.0, individual)
    fingers = hand_state()
    assert fingers[0] < 0.1, f"right pinky did not close: {fingers[0]}"
    np.testing.assert_array_less(0.85, fingers[[1, 2, 3, 6, 7, 8, 9]])

    initial_shoulder = state["lowstate"].motor_state[LEFT_SHOULDER_PITCH].q
    shoulder_target = initial_shoulder - 0.25
    drive(3.0, 1.0, shoulder_target)
    final_shoulder = state["lowstate"].motor_state[LEFT_SHOULDER_PITCH].q
    assert abs(final_shoulder - initial_shoulder) > 0.1, "arm command caused no motion"
    assert abs(final_shoulder - shoulder_target) < 0.1, (
        f"shoulder target {shoulder_target:.3f}, got {final_shoulder:.3f}"
    )

    print("PASS: body DDS, 12-hand DDS, startup hold, range, midpoint, and index mapping")
    print(f"PASS: left shoulder {initial_shoulder:.3f} -> {final_shoulder:.3f}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
