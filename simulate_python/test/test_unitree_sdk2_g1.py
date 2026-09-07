import time
from unitree_sdk2py.core.channel import ChannelFactoryInitialize
from unitree_sdk2py.core.channel import ChannelPublisher, ChannelSubscriber
from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowCmd_
from unitree_sdk2py.idl.unitree_go.msg.dds_ import SportModeState_
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_
from unitree_sdk2py.utils.crc import CRC

G1_NUM_MOTOR = 29


def HighStateHandler(msg: SportModeState_):
    print("Position: ", msg.position)


def LowStateHandler(msg: LowState_):
    print("IMU state: ", msg.imu_state)


if __name__ == "__main__":
    ChannelFactoryInitialize(1, "lo")
    high_state_suber = ChannelSubscriber("rt/sportmodestate", SportModeState_)
    low_state_suber = ChannelSubscriber("rt/lowstate", LowState_)

    high_state_suber.Init(HighStateHandler, 10)
    low_state_suber.Init(LowStateHandler, 10)

    low_cmd_puber = ChannelPublisher("rt/lowcmd", LowCmd_)
    low_cmd_puber.Init()
    crc = CRC()

    cmd = unitree_hg_msg_dds__LowCmd_()
    cmd.mode_pr = 0  # PR mode
    cmd.mode_machine = 0

    while True:
        for i in range(G1_NUM_MOTOR):
            cmd.motor_cmd[i].mode = 0x01
            cmd.motor_cmd[i].q = 0.0
            cmd.motor_cmd[i].kp = 0.0
            cmd.motor_cmd[i].dq = 0.0
            cmd.motor_cmd[i].kd = 0.0
            cmd.motor_cmd[i].tau = 1.0

        cmd.crc = crc.Crc(cmd)
        low_cmd_puber.Write(cmd)
        time.sleep(0.002)
