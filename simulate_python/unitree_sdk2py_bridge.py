import mujoco
import numpy as np

from unitree_sdk2py.core.channel import ChannelSubscriber, ChannelPublisher
from unitree_sdk2py.idl.unitree_go.msg.dds_ import LowCmd_
from unitree_sdk2py.idl.unitree_go.msg.dds_ import LowState_
from unitree_sdk2py.idl.unitree_go.msg.dds_ import SportModeState_
from unitree_sdk2py.idl.default import unitree_go_msg_dds__LowCmd_
from unitree_sdk2py.idl.default import unitree_go_msg_dds__LowState_
from unitree_sdk2py.idl.default import unitree_go_msg_dds__SportModeState_
from unitree_sdk2py.utils.thread import RecurrentThread

TOPIC_LOWCMD = "rt/lowcmd"
TOPIC_LOWSTATE = "rt/lowstate"
TOPIC_HIGHSTATE = "rt/sportmodestate"

MOTOR_SENSOR_NUM = 3


class UnitreeSdk2Bridge:

    def __init__(self, mj_model, mj_data, info):
        self.mj_model = mj_model
        self.mj_data = mj_data

        self.num_motor = self.mj_model.nu
        self.dim_motor_sensor = MOTOR_SENSOR_NUM * self.num_motor
        self.have_imu = False
        self.have_frame_sensor = False
        self.dt = self.mj_model.opt.timestep

        # Check sensor
        for i in range(self.dim_motor_sensor, self.mj_model.nsensor):
            name = mujoco.mj_id2name(self.mj_model,
                                     mujoco._enums.mjtObj.mjOBJ_SENSOR, i)
            if name == "imu_quat":
                self.have_imu_ = True
            if name == "frame_pos":
                self.have_frame_sensor_ = True

        if info:
            self.PrintSceneInformation()

        # Unitree sdk2 message
        self.low_state = unitree_go_msg_dds__LowState_()
        self.low_state_puber = ChannelPublisher(TOPIC_LOWSTATE, LowState_)
        self.low_state_puber.Init()
        self.lowStateThread = RecurrentThread(interval=0.002,
                                              target=self.PublishLowState,
                                              name="sim_pub_lowstate")
        self.lowStateThread.Start()

        self.high_state = unitree_go_msg_dds__SportModeState_()
        self.high_state_puber = ChannelPublisher(TOPIC_HIGHSTATE,
                                                 SportModeState_)
        self.high_state_puber.Init()
        self.HighStateThread = RecurrentThread(interval=self.dt,
                                               target=self.PublishHighState,
                                               name="highstate")
        self.HighStateThread.Start()

        self.low_cmd_suber = ChannelSubscriber(TOPIC_LOWCMD, LowCmd_)
        self.low_cmd_suber.Init(self.LowCmdHandler, 10)

    def LowCmdHandler(self, msg: LowCmd_):
        if self.mj_data != None:
            for i in range(self.num_motor):
                self.mj_data.ctrl[i] = msg.motor_cmd[i].tau +\
                                msg.motor_cmd[i].kp * (msg.motor_cmd[i].q - self.mj_data.sensordata[i]) +\
                                msg.motor_cmd[i].kd * (msg.motor_cmd[i].dq - self.mj_data.sensordata[i + self.num_motor])

    def PublishLowState(self):
        if self.mj_data != None:
            for i in range(self.num_motor):
                self.low_state.motor_state[i].q = self.mj_data.sensordata[i]
                self.low_state.motor_state[i].dq = self.mj_data.sensordata[
                    i + self.num_motor]
                self.low_state.motor_state[
                    i].tau_est = self.mj_data.sensordata[i +
                                                         2 * self.num_motor]

            if self.have_frame_sensor_:

                self.low_state.imu_state.quaternion[
                    0] = self.mj_data.sensordata[self.dim_motor_sensor + 0]
                self.low_state.imu_state.quaternion[
                    1] = self.mj_data.sensordata[self.dim_motor_sensor + 1]
                self.low_state.imu_state.quaternion[
                    2] = self.mj_data.sensordata[self.dim_motor_sensor + 2]
                self.low_state.imu_state.quaternion[
                    3] = self.mj_data.sensordata[self.dim_motor_sensor + 3]

                self.low_state.imu_state.gyroscope[
                    0] = self.mj_data.sensordata[self.dim_motor_sensor + 4]
                self.low_state.imu_state.gyroscope[
                    1] = self.mj_data.sensordata[self.dim_motor_sensor + 5]
                self.low_state.imu_state.gyroscope[
                    2] = self.mj_data.sensordata[self.dim_motor_sensor + 6]

                self.low_state.imu_state.accelerometer[
                    0] = self.mj_data.sensordata[self.dim_motor_sensor + 7]
                self.low_state.imu_state.accelerometer[
                    1] = self.mj_data.sensordata[self.dim_motor_sensor + 8]
                self.low_state.imu_state.accelerometer[
                    2] = self.mj_data.sensordata[self.dim_motor_sensor + 9]

            self.low_state_puber.Write(self.low_state)

    def PublishHighState(self):

        if self.mj_data != None:
            self.high_state.position[0] = self.mj_data.sensordata[
                self.dim_motor_sensor + 10]
            self.high_state.position[1] = self.mj_data.sensordata[
                self.dim_motor_sensor + 11]
            self.high_state.position[2] = self.mj_data.sensordata[
                self.dim_motor_sensor + 12]

            self.high_state.velocity[0] = self.mj_data.sensordata[
                self.dim_motor_sensor + 13]
            self.high_state.velocity[1] = self.mj_data.sensordata[
                self.dim_motor_sensor + 14]
            self.high_state.velocity[2] = self.mj_data.sensordata[
                self.dim_motor_sensor + 15]

        self.high_state_puber.Write(self.high_state)

    def PrintSceneInformation(self):
        print(" ")

        print("<<------------- Link ------------->> ")
        for i in range(self.mj_model.nbody):
            name = mujoco.mj_id2name(self.mj_model,
                                     mujoco._enums.mjtObj.mjOBJ_BODY, i)
            if name:
                print("link_index:", i, ", name:", name)
        print(" ")

        print("<<------------- Joint ------------->> ")
        for i in range(self.mj_model.njnt):
            name = mujoco.mj_id2name(self.mj_model,
                                     mujoco._enums.mjtObj.mjOBJ_JOINT, i)
            if name:
                print("joint_index:", i, ", name:", name)
        print(" ")

        print("<<------------- Actuator ------------->>")
        for i in range(self.mj_model.nu):
            name = mujoco.mj_id2name(self.mj_model,
                                     mujoco._enums.mjtObj.mjOBJ_ACTUATOR, i)
            if name:
                print("actuator_index:", i, ", name:", name)
        print(" ")

        print("<<------------- Sensor ------------->>")
        index = 0
        for i in range(self.mj_model.nsensor):
            name = mujoco.mj_id2name(self.mj_model,
                                     mujoco._enums.mjtObj.mjOBJ_SENSOR, i)
            if name:
                print("sensor_index:", index, ", name:", name, ", dim:",
                      self.mj_model.sensor_dim[i])
            index = index + self.mj_model.sensor_dim[i]
        print(" ")


class ElasticBand:

    def __init__(self):
        self.stiffness = 200
        self.damping = 100
        self.point = np.array([0, 0, 3])
        self.length = 0
        self.enable = True

    def Advance(self, x, dx):
        """
    Args:
      δx: desired position - current position
      dx: current velocity
    """
        δx = self.point - x
        distance = np.linalg.norm(δx)
        direction = δx / distance
        v = np.dot(dx, direction)
        f = (self.stiffness *
             (distance - self.length) - self.damping * v) * direction
        return f

    def MujuocoKeyCallback(self, key):
        glfw = mujoco.glfw.glfw
        if key == glfw.KEY_7:
            self.length -= 0.1
        if key == glfw.KEY_8:
            self.length += 0.1
        if key == glfw.KEY_9:
            self.enable = not self.enable
