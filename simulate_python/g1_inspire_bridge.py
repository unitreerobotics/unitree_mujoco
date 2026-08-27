"""DDS <-> MuJoCo bridge for G1 29-DoF body + Inspire hands.

Topics (matching xr_teleoperate and the real robot):
- subscribe rt/lowcmd      (unitree_hg LowCmd_)   : body joint targets (PD)
- publish   rt/lowstate    (unitree_hg LowState_) : body joint states + IMU
- subscribe rt/inspire/cmd (unitree_go MotorCmds_): 12 normalized hand targets
- publish   rt/inspire/state (unitree_go MotorStates_): 12 normalized states

Assumptions about the MuJoCo model (scene_29dof_inspire_fixed.xml):
- actuators 0..28  : body torque motors in Unitree G1 DDS joint order
- actuators 29..40 : hand position actuators in Inspire DDS id order
  (0-5 right pinky/ring/middle/index/thumb-bend/thumb-yaw, 6-11 left)
- jointpos/jointvel/jointactuatorfrc + IMU sensors exist for the body joints
"""

import threading

import mujoco
import numpy as np

from unitree_sdk2py.core.channel import ChannelPublisher, ChannelSubscriber
from unitree_sdk2py.utils.thread import RecurrentThread
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_ as HGLowCmd_
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_ as HGLowState_
from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowState_
from unitree_sdk2py.idl.unitree_go.msg.dds_ import MotorCmds_, MotorStates_
from unitree_sdk2py.idl.default import (
    unitree_go_msg_dds__MotorCmd_,
    unitree_go_msg_dds__MotorState_,
)
from unitree_sdk2py.utils.crc import CRC

TOPIC_LOWCMD = "rt/lowcmd"
TOPIC_LOWSTATE = "rt/lowstate"
TOPIC_INSPIRE_CMD = "rt/inspire/cmd"
TOPIC_INSPIRE_STATE = "rt/inspire/state"

NUM_BODY_MOTOR = 29
NUM_HAND_MOTOR = 12

# normalization ranges per Inspire DDS id (q_norm = (max - q) / (max - min))
INSPIRE_RANGES = np.array(
    [[0.0, 1.7]] * 4 + [[0.0, 0.5], [-0.1, 1.3]]
    + [[0.0, 1.7]] * 4 + [[0.0, 0.5], [-0.1, 1.3]]
)

# hold gains used for joints until a lowcmd arrives (keeps legs from dangling)
HOLD_KP = 60.0
HOLD_KD = 1.5


class G1InspireBridge:
    def __init__(self, mj_model, mj_data, data_lock):
        self.mj_model = mj_model
        self.mj_data = mj_data
        self.data_lock = data_lock
        self.lock = threading.Lock()
        self.crc = CRC()

        assert mj_model.nu == NUM_BODY_MOTOR + NUM_HAND_MOTOR, (
            f"expected {NUM_BODY_MOTOR + NUM_HAND_MOTOR} actuators, got {mj_model.nu}"
        )

        # --- resolve body joint addresses from actuator order ---
        self.body_qadr = np.zeros(NUM_BODY_MOTOR, dtype=int)
        self.body_dqadr = np.zeros(NUM_BODY_MOTOR, dtype=int)
        for i in range(NUM_BODY_MOTOR):
            jid = mj_model.actuator_trnid[i, 0]
            self.body_qadr[i] = mj_model.jnt_qposadr[jid]
            self.body_dqadr[i] = mj_model.jnt_dofadr[jid]

        # --- hand joint addresses (actuators 29..40, inspire DDS order) ---
        self.hand_qadr = np.zeros(NUM_HAND_MOTOR, dtype=int)
        for i in range(NUM_HAND_MOTOR):
            jid = mj_model.actuator_trnid[NUM_BODY_MOTOR + i, 0]
            self.hand_qadr[i] = mj_model.jnt_qposadr[jid]

        # --- IMU sensor addresses (by name, robust to layout changes) ---
        def sadr(name):
            sid = mujoco.mj_name2id(mj_model, mujoco.mjtObj.mjOBJ_SENSOR, name)
            return mj_model.sensor_adr[sid] if sid >= 0 else -1

        self.imu_quat_adr = sadr("imu_quat")
        self.imu_gyro_adr = sadr("imu_gyro")
        self.imu_acc_adr = sadr("imu_acc")

        # --- body command state (hold current pose until lowcmd arrives) ---
        self.cmd_q = self.mj_data.qpos[self.body_qadr].copy()
        self.cmd_dq = np.zeros(NUM_BODY_MOTOR)
        self.cmd_tau = np.zeros(NUM_BODY_MOTOR)
        self.cmd_kp = np.full(NUM_BODY_MOTOR, HOLD_KP)
        self.cmd_kd = np.full(NUM_BODY_MOTOR, HOLD_KD)
        self.lowcmd_received = False

        # Hold the model's initial pose until the first complete DDS command.
        self.hand_target = self.mj_data.qpos[self.hand_qadr].copy()
        self.inspire_command_received = False

        # --- DDS pub/sub ---
        self.low_state = unitree_hg_msg_dds__LowState_()
        self.low_state_puber = ChannelPublisher(TOPIC_LOWSTATE, HGLowState_)
        self.low_state_puber.Init()

        self.inspire_state = MotorStates_()
        self.inspire_state.states = [
            unitree_go_msg_dds__MotorState_() for _ in range(NUM_HAND_MOTOR)
        ]
        self.inspire_state_puber = ChannelPublisher(TOPIC_INSPIRE_STATE, MotorStates_)
        self.inspire_state_puber.Init()

        self.low_cmd_suber = ChannelSubscriber(TOPIC_LOWCMD, HGLowCmd_)
        self.low_cmd_suber.Init(self.LowCmdHandler, 10)

        self.inspire_cmd_suber = ChannelSubscriber(TOPIC_INSPIRE_CMD, MotorCmds_)
        self.inspire_cmd_suber.Init(self.InspireCmdHandler, 10)

        dt = mj_model.opt.timestep
        self.low_state_thread = RecurrentThread(
            interval=max(dt, 0.002), target=self.PublishLowState, name="sim_lowstate"
        )
        self.low_state_thread.Start()
        self.inspire_state_thread = RecurrentThread(
            interval=0.01, target=self.PublishInspireState, name="sim_inspire_state"
        )
        self.inspire_state_thread.Start()

    # ------------------------------------------------------------- subscribe
    def LowCmdHandler(self, msg: HGLowCmd_):
        with self.lock:
            for i in range(NUM_BODY_MOTOR):
                mc = msg.motor_cmd[i]
                self.cmd_q[i] = mc.q
                self.cmd_dq[i] = mc.dq
                self.cmd_tau[i] = mc.tau
                self.cmd_kp[i] = mc.kp
                self.cmd_kd[i] = mc.kd
            self.lowcmd_received = True

    def InspireCmdHandler(self, msg: MotorCmds_):
        if len(msg.cmds) != NUM_HAND_MOTOR:
            return
        target = np.empty(NUM_HAND_MOTOR)
        for i, (lo, hi) in enumerate(INSPIRE_RANGES):
            q_norm = np.clip(msg.cmds[i].q, 0.0, 1.0)
            # q_norm: 1.0 = fully open (q=lo), 0.0 = fully closed (q=hi)
            target[i] = hi - q_norm * (hi - lo)
        with self.lock:
            self.hand_target[:] = target
            self.inspire_command_received = True

    # ------------------------------------------------------------------ step
    def update_ctrl(self):
        """Call once per physics step (with the sim lock held by the caller)."""
        d = self.mj_data
        q = d.qpos[self.body_qadr]
        dq = d.qvel[self.body_dqadr]
        with self.lock:
            tau = (
                self.cmd_tau
                + self.cmd_kp * (self.cmd_q - q)
                + self.cmd_kd * (self.cmd_dq - dq)
            )
            d.ctrl[:NUM_BODY_MOTOR] = tau
            d.ctrl[NUM_BODY_MOTOR:NUM_BODY_MOTOR + NUM_HAND_MOTOR] = self.hand_target

    # ------------------------------------------------------------- publish
    def PublishLowState(self):
        d = self.mj_data
        with self.data_lock:
            q = d.qpos[self.body_qadr].copy()
            dq = d.qvel[self.body_dqadr].copy()
            force = d.actuator_force[:NUM_BODY_MOTOR].copy()
            sensors = d.sensordata.copy()
        for i in range(NUM_BODY_MOTOR):
            ms = self.low_state.motor_state[i]
            ms.q = q[i]
            ms.dq = dq[i]
            ms.tau_est = force[i]
        if self.imu_quat_adr >= 0:
            for k in range(4):
                self.low_state.imu_state.quaternion[k] = sensors[self.imu_quat_adr + k]
        if self.imu_gyro_adr >= 0:
            for k in range(3):
                self.low_state.imu_state.gyroscope[k] = sensors[self.imu_gyro_adr + k]
        if self.imu_acc_adr >= 0:
            for k in range(3):
                self.low_state.imu_state.accelerometer[k] = sensors[self.imu_acc_adr + k]
        self.low_state.tick += 1
        self.low_state.crc = self.crc.Crc(self.low_state)
        self.low_state_puber.Write(self.low_state)

    def PublishInspireState(self):
        with self.data_lock:
            q = self.mj_data.qpos[self.hand_qadr].copy()
        for i in range(NUM_HAND_MOTOR):
            lo, hi = INSPIRE_RANGES[i]
            self.inspire_state.states[i].q = float(np.clip((hi - q[i]) / (hi - lo), 0.0, 1.0))
        self.inspire_state_puber.Write(self.inspire_state)

    def close(self):
        self.low_state_thread.Wait(1.0)
        self.inspire_state_thread.Wait(1.0)
