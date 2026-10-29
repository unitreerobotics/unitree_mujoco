"""
Run the G1 with a dexterous hand attached (pelvis welded to the world by default).

Body: rt/lowcmd and rt/lowstate (29 motors). Hands: their own DDS topics, see g1_hands_bridge.py.

    python g1_hands_run.py                          # Dex3 (default)
    python g1_hands_run.py --hand brainco           # also: dex1, inspire_ftp, inspire_dfx
    python g1_hands_run.py --hand dex1 --free-base --no-band
"""

import argparse
import time
import threading
from threading import Thread

import mujoco
import mujoco.viewer

from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelPublisher, ChannelSubscriber
from unitree_sdk2py.idl.default import unitree_go_msg_dds__SportModeState_, unitree_hg_msg_dds__LowState_
from unitree_sdk2py.idl.unitree_go.msg.dds_ import SportModeState_
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_, LowState_
from unitree_sdk2py.utils.thread import RecurrentThread

from g1_hands_bridge import BrainCoBridge, Dex1Bridge, Dex3Bridge, InspireDFXBridge, InspireFTPBridge
from unitree_sdk2py_bridge import ElasticBand

DOMAIN_ID = 1
INTERFACE = "lo"
NUM_BODY_MOTOR = 29  # LowCmd_/LowState_ slots used by the G1 body

ROBOTS_DIR = "../unitree_robots/g1/"
HANDS = {
    "dex3": "scene_29dof_dex3_fixed_base.xml",
    "dex1": "scene_29dof_dex1_fixed_base.xml",
    "inspire_ftp": "scene_29dof_inspire_ftp_fixed_base.xml",
    "inspire_dfx": "scene_29dof_inspire_dfx_fixed_base.xml",
    "brainco": "scene_29dof_brainco_fixed_base.xml",
}
# Actuator names in the motor order each protocol uses. Inspire DFX: right hand first.
DFX_ORDER = ["pinky_proximal", "ring_proximal", "middle_proximal", "index_proximal", "thumb_proximal_pitch", "thumb_proximal_yaw"]
FTP_ORDER = ["little_1", "ring_1", "middle_1", "index_1", "thumb_2", "thumb_1"]  
BRAINCO_ORDER = ["thumb_proximal", "thumb_metacarpal", "index_proximal", "middle_proximal", "ring_proximal", "pinky_proximal"]

parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
parser.add_argument("--hand", choices=list(HANDS), default="dex3")
parser.add_argument("--domain-id", type=int, default=DOMAIN_ID, help="DDS domain (change it to run a second sim)")
parser.add_argument("--no-band", action="store_true", help="start with the elastic band off (it pulls the torso up; key 9 toggles it)")
parser.add_argument("--free-base", action="store_true", help="do not weld the pelvis to the world (no balance controller: the robot falls unless the band is on)")
parser.add_argument("--dt", type=float, help="physics step in seconds (default: the scene's, 0.002)")
parser.add_argument("--stand", action="store_true", help="start standing and freeze the physics until the first rt/lowcmd arrives (for --free-base with a balance controller)")
args = parser.parse_args()

locker = threading.Lock()

scene = HANDS[args.hand].replace("_fixed_base", "") if args.free_base else HANDS[args.hand]
mj_model = mujoco.MjModel.from_xml_path(ROBOTS_DIR + scene)
if args.dt:
    mj_model.opt.timestep = args.dt
mj_data = mujoco.MjData(mj_model)
if args.stand:  # standing pose of legs and waist, pelvis just above the floor
    mj_data.qpos[2] = 0.78
    for i, angle in enumerate([-0.1, 0, 0, 0.3, -0.2, 0] * 2 + [0, 0, 0]):
        mj_data.qpos[mj_model.jnt_qposadr[mj_model.actuator_trnid[i, 0]]] = angle
    mujoco.mj_forward(mj_model, mj_data)

# DDS motor index -> actuator index (identity: the 29 body motors come first in the models).
BODY_MAP = {i: i for i in range(NUM_BODY_MOTOR)}
N_BODY_ACT = len(BODY_MAP)


def actuator_id(name: str) -> int:
    idx = mujoco.mj_name2id(mj_model, mujoco.mjtObj.mjOBJ_ACTUATOR, name)
    assert idx >= 0, f"actuator '{name}' not in {HANDS[args.hand]}"
    return idx


elastic_band = ElasticBand()
elastic_band.enable = not args.no_band
band_attached_link = mj_model.body("torso_link").id
viewer = mujoco.viewer.launch_passive(mj_model, mj_data, key_callback=elastic_band.MujuocoKeyCallback)


class BodyBridge29:
    """LowCmd_/LowState_ bridge for the 29 body motors only. UnitreeSdk2Bridge loops over mj_model.nu,
    which here includes the hand motors and would pass LowCmd_'s 35 slots."""

    def __init__(self, mj_model, mj_data):
        self.mj_model = mj_model
        self.mj_data = mj_data
        self.got_cmd = False
        self.low_state = unitree_hg_msg_dds__LowState_()
        self.low_state_puber = ChannelPublisher("rt/lowstate", LowState_)
        self.low_state_puber.Init()
        self.low_cmd_suber = ChannelSubscriber("rt/lowcmd", LowCmd_)
        self.low_cmd_suber.Init(self.LowCmdHandler, 10)
        # pelvis IMU, by sensor name (they come after the motor sensors)
        self.imu_adr = {n: int(mj_model.sensor(n).adr[0]) for n in ("imu_quat", "imu_gyro", "imu_acc", "frame_pos", "frame_vel")}
        # base position and velocity, on the same topic as the official bridge
        self.base_state = unitree_go_msg_dds__SportModeState_()
        self.base_state_puber = ChannelPublisher("rt/sportmodestate", SportModeState_)
        self.base_state_puber.Init()
        # 100 Hz
        self.state_thread = RecurrentThread(
            interval=0.01, target=self.PublishLowState, name="g1_hands_lowstate"
        )
        self.state_thread.Start()

    def LowCmdHandler(self, msg: LowCmd_):
        self.got_cmd = True
        for dds_i, act in BODY_MAP.items():
            q = self.mj_data.sensordata[act]
            dq = self.mj_data.sensordata[act + mj_model.nu]
            self.mj_data.ctrl[act] = (
                msg.motor_cmd[dds_i].tau
                + msg.motor_cmd[dds_i].kp * (msg.motor_cmd[dds_i].q - q)
                + msg.motor_cmd[dds_i].kd * (msg.motor_cmd[dds_i].dq - dq)
            )

    def PublishLowState(self):
        for dds_i, act in BODY_MAP.items():
            self.low_state.motor_state[dds_i].q = self.mj_data.sensordata[act]
            self.low_state.motor_state[dds_i].dq = self.mj_data.sensordata[act + mj_model.nu]
            self.low_state.motor_state[dds_i].tau_est = self.mj_data.sensordata[act + 2 * mj_model.nu]
        imu, sd = self.low_state.imu_state, self.mj_data.sensordata
        for field, name, n in (("quaternion", "imu_quat", 4), ("gyroscope", "imu_gyro", 3), ("accelerometer", "imu_acc", 3)):
            a = self.imu_adr[name]
            for k in range(n):
                getattr(imu, field)[k] = float(sd[a + k])
        self.low_state_puber.Write(self.low_state)
        for field, name in (("position", "frame_pos"), ("velocity", "frame_vel")):
            a = self.imu_adr[name]
            for k in range(3):
                getattr(self.base_state, field)[k] = float(sd[a + k])
        self.base_state_puber.Write(self.base_state)


def make_hand_bridges():
    if args.hand == "dex3":
        assert mj_model.nu == N_BODY_ACT + 14, f"expected {N_BODY_ACT} body + 14 hand actuators, got {mj_model.nu}"
        return [Dex3Bridge(mj_model, mj_data, "left", N_BODY_ACT), Dex3Bridge(mj_model, mj_data, "right", N_BODY_ACT + 7)]
    if args.hand == "inspire_dfx":
        acts = [actuator_id(f"{side}_{n}_joint") for side in ("R", "L") for n in DFX_ORDER]
        return [InspireDFXBridge(mj_model, mj_data, acts)]
    bridges = []
    for side in ("left", "right"):
        if args.hand == "dex1":
            acts = [actuator_id(f"{side}_dex1_finger_joint_{k}") for k in (1, 2)]
            bridges.append(Dex1Bridge(mj_model, mj_data, side, acts))
        elif args.hand == "inspire_ftp":
            acts = [actuator_id(f"{side}_{n}_joint") for n in FTP_ORDER]
            bridges.append(InspireFTPBridge(mj_model, mj_data, side, acts))
        elif args.hand == "brainco":
            acts = [actuator_id(f"{side}_{n}_joint") for n in BRAINCO_ORDER]
            bridges.append(BrainCoBridge(mj_model, mj_data, side, acts))
    return bridges


def simulation_thread():
    ChannelFactoryInitialize(args.domain_id, INTERFACE)
    body_bridge = BodyBridge29(mj_model, mj_data)
    hand_bridges = make_hand_bridges()

    while viewer.is_running():
        step_start = time.perf_counter()
        if args.stand and not body_bridge.got_cmd:  # wait for a controller
            time.sleep(0.01)
            continue

        locker.acquire()
        if elastic_band.enable:
            mj_data.xfrc_applied[band_attached_link, :3] = elastic_band.Advance(
                mj_data.qpos[:3], mj_data.qvel[:3]
            )
        else:
            mj_data.xfrc_applied[band_attached_link, :3] = 0.0  # band off: clear the last force
        for hand in hand_bridges:
            hand.apply_ctrl()
        mujoco.mj_step(mj_model, mj_data)
        locker.release()

        elapsed = time.perf_counter() - step_start
        remaining = mj_model.opt.timestep - elapsed
        if remaining > 0:
            time.sleep(remaining)


def viewer_thread():
    while viewer.is_running():
        locker.acquire()
        viewer.sync()
        locker.release()
        time.sleep(0.02)


if __name__ == "__main__":
    Thread(target=viewer_thread).start()
    Thread(target=simulation_thread).start()
