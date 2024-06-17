import time
from unitree_sdk2py.core.channel import ChannelFactoryInitialize
from unitree_sdk2py.core.channel import ChannelPublisher, ChannelSubscriber
from unitree_sdk2py.idl.default import unitree_go_msg_dds__SportModeState_
from unitree_sdk2py.idl.default import unitree_go_msg_dds__LowState_
from unitree_sdk2py.idl.default import unitree_go_msg_dds__LowCmd_
from unitree_sdk2py.idl.unitree_go.msg.dds_ import SportModeState_
from unitree_sdk2py.idl.unitree_go.msg.dds_ import LowState_
from unitree_sdk2py.idl.unitree_go.msg.dds_ import LowCmd_
from unitree_sdk2py.utils.crc import CRC


def HighStateHandler(msg: SportModeState_):
    print("Position: ", msg.position)
    #print("Velocity: ", msg.velocity)


def LowStateHandler(msg: LowState_):
    print("IMU state: ", msg.imu_state)
    # print("motor[0] state: ", msg.motor_state[0])


if __name__ == "__main__":
    ChannelFactoryInitialize(1, "lo")
    hight_state_suber = ChannelSubscriber("rt/sportmodestate", SportModeState_)
    low_state_suber = ChannelSubscriber("rt/lowstate", LowState_)

    hight_state_suber.Init(HighStateHandler, 10)
    low_state_suber.Init(LowStateHandler, 10)

    low_cmd_puber = ChannelPublisher("rt/lowcmd", LowCmd_)
    low_cmd_puber.Init()
    crc = CRC()

    cmd = unitree_go_msg_dds__LowCmd_()
    cmd.head[0]=0xFE
    cmd.head[1]=0xEF
    cmd.level_flag = 0xFF
    cmd.gpio = 0
    for i in range(20):
        cmd.motor_cmd[i].mode = 0x01  # (PMSM) mode
        cmd.motor_cmd[i].q= 0.0
        cmd.motor_cmd[i].kp = 0.0
        cmd.motor_cmd[i].dq = 0.0
        cmd.motor_cmd[i].kd = 0.0
        cmd.motor_cmd[i].tau = 0.0
        
    while True:
        for i in range(12):
            cmd.motor_cmd[i].q = 0.0 
            cmd.motor_cmd[i].kp = 0.0
            cmd.motor_cmd[i].dq = 0.0 
            cmd.motor_cmd[i].kd = 0.0
            cmd.motor_cmd[i].tau = 1.0 
        
        cmd.crc = crc.Crc(cmd)

        #Publish message
        low_cmd_puber.Write(cmd)
        time.sleep(0.002)
