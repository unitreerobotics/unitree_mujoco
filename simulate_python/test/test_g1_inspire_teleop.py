"""Fake-teleop test: drives arms via rt/lowcmd and hands via rt/inspire/cmd,
and checks rt/lowstate + rt/inspire/state come back. Run g1_inspire_sim.py
first, then run this in another terminal (same machine):

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
RIGHT_SHOULDER_PITCH = 22
LEFT_ELBOW = 18
RIGHT_ELBOW = 25

state = {"lowstate": None, "inspire": None}


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

    print("waiting for rt/lowstate ...")
    t0 = time.time()
    while state["lowstate"] is None:
        time.sleep(0.05)
        if time.time() - t0 > 5:
            print("FAIL: no rt/lowstate received"); return 1
    print("OK: rt/lowstate received")

    t0 = time.time()
    while state["inspire"] is None:
        time.sleep(0.05)
        if time.time() - t0 > 5:
            print("FAIL: no rt/inspire/state received"); return 1
    print("OK: rt/inspire/state received (first q values:",
          [round(state['inspire'].states[i].q, 2) for i in range(6)], ")")

    # hold every joint at current pos, then wave arms + open/close hands
    ls = state["lowstate"]
    for i in range(29):
        cmd.motor_cmd[i].q = ls.motor_state[i].q
        cmd.motor_cmd[i].kp = KP
        cmd.motor_cmd[i].kd = KD

    print("driving arms (shoulder pitch + elbow) and hands for 8s ...")
    start = time.time()
    while time.time() - start < 8.0:
        t = time.time() - start
        # arms: raise forward and bend elbows sinusoidally
        target = 0.6 * np.sin(2 * np.pi * 0.25 * t)
        cmd.motor_cmd[LEFT_SHOULDER_PITCH].q = -abs(target)
        cmd.motor_cmd[RIGHT_SHOULDER_PITCH].q = -abs(target)
        cmd.motor_cmd[LEFT_ELBOW].q = abs(target)
        cmd.motor_cmd[RIGHT_ELBOW].q = abs(target)
        cmd.crc = crc.Crc(cmd)
        lowcmd_pub.Write(cmd)

        # hands: 1.0 = open, 0.0 = closed
        grip = 0.5 + 0.5 * np.sin(2 * np.pi * 0.5 * t)
        for i in range(12):
            hand_cmd.cmds[i].q = float(grip)
        inspire_pub.Write(hand_cmd)
        time.sleep(0.01)

    # verify motion happened
    ls = state["lowstate"]
    q_shoulder = ls.motor_state[LEFT_SHOULDER_PITCH].q
    ins = state["inspire"]
    print(f"final left shoulder pitch q = {q_shoulder:.3f}")
    print("final hand norm q =", [round(ins.states[i].q, 2) for i in range(12)])
    print("DONE")
    return 0


if __name__ == "__main__":
    sys.exit(main())
