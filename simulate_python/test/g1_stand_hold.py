"""
Hold G1 in a stable standing pose (for unitree_mujoco simulation).

Usage:
  1. Start the simulator (G1 loaded, elastic band ON).
  2. Run:  python3 g1_stand_hold.py
  3. In the sim window: press 8 to lift, 7 to lower until feet touch ground,
     then press 9 to release the band. The robot holds its pose.
"""
import time
import numpy as np

from unitree_sdk2py.core.channel import ChannelPublisher, ChannelSubscriber
from unitree_sdk2py.core.channel import ChannelFactoryInitialize
from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowCmd_
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_, LowState_
from unitree_sdk2py.utils.crc import CRC

G1_NUM_MOTOR = 29
CONTROL_DT = 0.002
RAMP_TIME = 3.0  # seconds to move from current pose to stand pose

Kp = [
    100, 100, 100, 150, 40, 40,   # left leg
    100, 100, 100, 150, 40, 40,   # right leg
    100, 40, 40,                  # waist
    40, 40, 40, 40, 40, 40, 40,   # left arm
    40, 40, 40, 40, 40, 40, 40,   # right arm
]
Kd = [
    2, 2, 2, 4, 2, 2,
    2, 2, 2, 4, 2, 2,
    2, 1, 1,
    1, 1, 1, 1, 1, 1, 1,
    1, 1, 1, 1, 1, 1, 1,
]

# Slight crouch: hips/knees/ankles bent so the robot is statically stable
stand_pose = np.zeros(G1_NUM_MOTOR)
stand_pose[0] = -0.2   # LeftHipPitch
stand_pose[3] = 0.42   # LeftKnee
stand_pose[4] = -0.23  # LeftAnklePitch
stand_pose[6] = -0.2   # RightHipPitch
stand_pose[9] = 0.42   # RightKnee
stand_pose[10] = -0.23 # RightAnklePitch

low_state = None


def LowStateHandler(msg: LowState_):
    global low_state
    low_state = msg


if __name__ == "__main__":
    ChannelFactoryInitialize(1, "lo")

    suber = ChannelSubscriber("rt/lowstate", LowState_)
    suber.Init(LowStateHandler, 10)
    puber = ChannelPublisher("rt/lowcmd", LowCmd_)
    puber.Init()
    crc = CRC()

    print("Waiting for robot state...")
    while low_state is None:
        time.sleep(0.1)
    print("Connected. Holding stand pose. Ctrl+C to stop.")

    start_pose = np.array([low_state.motor_state[i].q for i in range(G1_NUM_MOTOR)])
    cmd = unitree_hg_msg_dds__LowCmd_()
    cmd.mode_pr = 0  # PR mode
    cmd.mode_machine = low_state.mode_machine

    t = 0.0
    while True:
        ratio = np.clip(t / RAMP_TIME, 0.0, 1.0)
        target = (1.0 - ratio) * start_pose + ratio * stand_pose

        for i in range(G1_NUM_MOTOR):
            cmd.motor_cmd[i].mode = 1
            cmd.motor_cmd[i].q = float(target[i])
            cmd.motor_cmd[i].dq = 0.0
            cmd.motor_cmd[i].tau = 0.0
            cmd.motor_cmd[i].kp = Kp[i]
            cmd.motor_cmd[i].kd = Kd[i]

        cmd.crc = crc.Crc(cmd)
        puber.Write(cmd)
        t += CONTROL_DT
        time.sleep(CONTROL_DT)
