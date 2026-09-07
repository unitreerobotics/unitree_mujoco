import math
import struct
from dataclasses import dataclass, field
from pathlib import Path
from typing import Mapping

import numpy as np
import rclpy
from rclpy.node import Node
from unitree_go.msg import SportModeState, WirelessController
from unitree_hg.msg import BmsState, IMUState, LowCmd, LowState


REQUIRED_SENSOR_NAMES = (
    "imu_quat",
    "imu_gyro",
    "imu_acc",
    "secondary_imu_quat",
    "secondary_imu_gyro",
    "secondary_imu_acc",
)
OPTIONAL_SENSOR_NAMES = ("frame_pos", "frame_vel")
TOPIC_LOWCMD = "/lowcmd"
TOPIC_LOWSTATE = "/lowstate"
TOPIC_SECONDARY_IMU = "/secondary_imu"
TOPIC_BMSSTATE = "/lf/bmsstate"
TOPIC_WIRELESS_CONTROLLER = "/wirelesscontroller"
TOPIC_SPORTMODESTATE = "/sportmodestate"
NUM_HG_MOTOR_SLOTS = 35
NUM_HG_WIRELESS_REMOTE_BYTES = 40


def _canonicalize_axis(value: float) -> float:
    return 0.0 if abs(value) < 1e-12 else float(value)


def infer_mode_machine(scene_path: str) -> int:
    return 4 if "23" in Path(scene_path).name else 5


def quat_to_rpy(quat_wxyz) -> tuple[float, float, float]:
    w, x, y, z = quat_wxyz
    roll = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
    sin_pitch = 2.0 * (w * y - z * x)
    sin_pitch = max(-1.0, min(1.0, sin_pitch))
    pitch = math.asin(sin_pitch)
    yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    return roll, pitch, yaw


def resolve_sensor_addresses(sensor_adr_by_name: Mapping[str, int]) -> dict[str, int | None]:
    missing = [name for name in REQUIRED_SENSOR_NAMES if name not in sensor_adr_by_name]
    if missing:
        raise ValueError("Missing required MuJoCo sensors: " + ", ".join(missing))
    resolved = {name: sensor_adr_by_name[name] for name in REQUIRED_SENSOR_NAMES}
    for name in OPTIONAL_SENSOR_NAMES:
        resolved[name] = sensor_adr_by_name.get(name)
    return resolved


def compute_motor_torques(low_cmd: LowCmd, joint_pos, joint_vel, num_motor: int) -> np.ndarray:
    torques = np.zeros(num_motor, dtype=np.float32)
    for i in range(num_motor):
        motor_cmd = low_cmd.motor_cmd[i]
        torques[i] = (
            motor_cmd.tau
            + motor_cmd.kp * (motor_cmd.q - joint_pos[i])
            + motor_cmd.kd * (motor_cmd.dq - joint_vel[i])
        )
    return torques


@dataclass
class GamepadSnapshot:
    axes: dict[str, float] = field(default_factory=dict)
    buttons: dict[str, bool] = field(default_factory=dict)
    hat: tuple[int, int] = (0, 0)


class GamepadAdapter:
    KEY_MAP = {
        "R1": 0,
        "L1": 1,
        "start": 2,
        "select": 3,
        "R2": 4,
        "L2": 5,
        "F1": 6,
        "F2": 7,
        "A": 8,
        "B": 9,
        "X": 10,
        "Y": 11,
        "up": 12,
        "right": 13,
        "down": 14,
        "left": 15,
    }

    JOYSTICK_LAYOUTS = {
        "xbox": {
            "axes": {"LX": 0, "LY": 1, "RX": 3, "RY": 4, "LT": 2, "RT": 5, "DX": 6, "DY": 7},
            "buttons": {"X": 2, "Y": 3, "B": 1, "A": 0, "LB": 4, "RB": 5, "SELECT": 6, "START": 7},
        },
        "switch": {
            "axes": {"LX": 0, "LY": 1, "RX": 2, "RY": 3, "LT": 5, "RT": 4, "DX": 6, "DY": 7},
            "buttons": {"X": 3, "Y": 4, "B": 1, "A": 0, "LB": 6, "RB": 7, "SELECT": 10, "START": 11},
        },
    }

    def __init__(self, use_joystick: bool, js_type: str = "xbox", device_id: int = 0):
        self._use_joystick = bool(use_joystick)
        self._joystick = None
        self._pygame = None
        self._layout = self.JOYSTICK_LAYOUTS.get(js_type)
        if self._layout is None:
            raise ValueError(f"Unsupported joystick type: {js_type}")
        if not self._use_joystick:
            return

        try:
            import pygame
        except ImportError:
            return

        pygame.init()
        pygame.joystick.init()
        if pygame.joystick.get_count() <= 0:
            self._pygame = pygame
            return
        self._pygame = pygame
        self._joystick = pygame.joystick.Joystick(device_id)
        self._joystick.init()

    def has_real_joystick(self) -> bool:
        return self._joystick is not None

    def snapshot(self) -> GamepadSnapshot:
        if self._pygame is not None:
            self._pygame.event.get()
        if self._joystick is None:
            return GamepadSnapshot()

        axes = {
            "LX": float(self._joystick.get_axis(self._layout["axes"]["LX"])),
            "LY": float(self._joystick.get_axis(self._layout["axes"]["LY"])),
            "RX": float(self._joystick.get_axis(self._layout["axes"]["RX"])),
            "RY": float(self._joystick.get_axis(self._layout["axes"]["RY"])),
            "LT": float(self._joystick.get_axis(self._layout["axes"]["LT"])),
            "RT": float(self._joystick.get_axis(self._layout["axes"]["RT"])),
        }
        buttons = {
            "A": bool(self._joystick.get_button(self._layout["buttons"]["A"])),
            "B": bool(self._joystick.get_button(self._layout["buttons"]["B"])),
            "X": bool(self._joystick.get_button(self._layout["buttons"]["X"])),
            "Y": bool(self._joystick.get_button(self._layout["buttons"]["Y"])),
            "LB": bool(self._joystick.get_button(self._layout["buttons"]["LB"])),
            "RB": bool(self._joystick.get_button(self._layout["buttons"]["RB"])),
            "SELECT": bool(self._joystick.get_button(self._layout["buttons"]["SELECT"])),
            "START": bool(self._joystick.get_button(self._layout["buttons"]["START"])),
        }
        return GamepadSnapshot(axes=axes, buttons=buttons, hat=self._joystick.get_hat(0))


class ElasticBand:
    def __init__(self):
        self.stiffness = 200
        self.damping = 100
        self.point = np.array([0, 0, 3], dtype=np.float32)
        self.length = 0
        self.enable = True

    def Advance(self, x, dx):
        delta = self.point - x
        distance = np.linalg.norm(delta)
        if distance <= 1e-8:
            return np.zeros(3, dtype=np.float32)
        direction = delta / distance
        velocity_along_direction = np.dot(dx, direction)
        force = (
            self.stiffness * (distance - self.length) - self.damping * velocity_along_direction
        ) * direction
        return force

    def MujuocoKeyCallback(self, key):
        import mujoco

        glfw = mujoco.glfw.glfw
        if key == glfw.KEY_7:
            self.length -= 0.1
        if key == glfw.KEY_8:
            self.length += 0.1
        if key == glfw.KEY_9:
            self.enable = not self.enable


def populate_wireless_controller_message(snapshot: GamepadSnapshot, msg: WirelessController | None = None) -> WirelessController:
    if msg is None:
        msg = WirelessController()
    key_state = [0] * 16
    key_state[GamepadAdapter.KEY_MAP["R1"]] = int(snapshot.buttons.get("RB", False))
    key_state[GamepadAdapter.KEY_MAP["L1"]] = int(snapshot.buttons.get("LB", False))
    key_state[GamepadAdapter.KEY_MAP["start"]] = int(snapshot.buttons.get("START", False))
    key_state[GamepadAdapter.KEY_MAP["select"]] = int(snapshot.buttons.get("SELECT", False))
    key_state[GamepadAdapter.KEY_MAP["R2"]] = int(snapshot.axes.get("RT", 0.0) > 0)
    key_state[GamepadAdapter.KEY_MAP["L2"]] = int(snapshot.axes.get("LT", 0.0) > 0)
    key_state[GamepadAdapter.KEY_MAP["A"]] = int(snapshot.buttons.get("A", False))
    key_state[GamepadAdapter.KEY_MAP["B"]] = int(snapshot.buttons.get("B", False))
    key_state[GamepadAdapter.KEY_MAP["X"]] = int(snapshot.buttons.get("X", False))
    key_state[GamepadAdapter.KEY_MAP["Y"]] = int(snapshot.buttons.get("Y", False))
    key_state[GamepadAdapter.KEY_MAP["up"]] = int(snapshot.hat[1] > 0)
    key_state[GamepadAdapter.KEY_MAP["right"]] = int(snapshot.hat[0] > 0)
    key_state[GamepadAdapter.KEY_MAP["down"]] = int(snapshot.hat[1] < 0)
    key_state[GamepadAdapter.KEY_MAP["left"]] = int(snapshot.hat[0] < 0)

    key_value = 0
    for i, value in enumerate(key_state):
        key_value += value << i

    msg.keys = key_value
    msg.lx = _canonicalize_axis(snapshot.axes.get("LX", 0.0))
    msg.ly = _canonicalize_axis(-snapshot.axes.get("LY", 0.0))
    msg.rx = _canonicalize_axis(snapshot.axes.get("RX", 0.0))
    msg.ry = _canonicalize_axis(-snapshot.axes.get("RY", 0.0))
    return msg


def encode_wireless_remote(snapshot: GamepadSnapshot) -> list[int]:
    data = [0] * NUM_HG_WIRELESS_REMOTE_BYTES
    data[2] = int(
        "".join(
            [
                "0",
                "0",
                str(int(snapshot.axes.get("LT", 0.0) > 0)),
                str(int(snapshot.axes.get("RT", 0.0) > 0)),
                str(int(snapshot.buttons.get("SELECT", False))),
                str(int(snapshot.buttons.get("START", False))),
                str(int(snapshot.buttons.get("LB", False))),
                str(int(snapshot.buttons.get("RB", False))),
            ]
        ),
        2,
    )
    data[3] = int(
        "".join(
            [
                str(int(snapshot.hat[0] < 0)),
                str(int(snapshot.hat[1] < 0)),
                str(int(snapshot.hat[0] > 0)),
                str(int(snapshot.hat[1] > 0)),
                str(int(snapshot.buttons.get("Y", False))),
                str(int(snapshot.buttons.get("X", False))),
                str(int(snapshot.buttons.get("B", False))),
                str(int(snapshot.buttons.get("A", False))),
            ]
        ),
        2,
    )
    sticks = [
        _canonicalize_axis(snapshot.axes.get("LX", 0.0)),
        _canonicalize_axis(snapshot.axes.get("RX", 0.0)),
        _canonicalize_axis(-snapshot.axes.get("RY", 0.0)),
        _canonicalize_axis(-snapshot.axes.get("LY", 0.0)),
    ]
    packs = [struct.pack("f", value) for value in sticks]
    data[4:8] = packs[0]
    data[8:12] = packs[1]
    data[12:16] = packs[2]
    data[20:24] = packs[3]
    return [int(v) for v in data]


class UnitreeRos2Bridge(Node):
    def __init__(self, mj_model, mj_data, scene_path: str, *, use_joystick: bool, joystick_type: str, joystick_device: int):
        super().__init__("unitree_mujoco_ros2_bridge")
        self.mj_model = mj_model
        self.mj_data = mj_data
        self.scene_path = scene_path
        self.num_motor = int(self.mj_model.nu)
        self.dim_motor_sensor = 3 * self.num_motor
        self.mode_machine = infer_mode_machine(scene_path)
        self.latest_low_cmd = LowCmd()
        self.latest_mode_pr = 0
        self.warned_missing_frame_sensor = False
        self.bms_state_msg = BmsState()
        self.bms_state_msg.soc = 100
        self.secondary_imu_msg = IMUState()
        self.low_state_msg = LowState()
        self.sport_state_msg = SportModeState()
        self.wireless_controller_msg = WirelessController()
        self.sensor_adr = resolve_sensor_addresses(self._build_sensor_address_map())
        self.gamepad = GamepadAdapter(use_joystick, joystick_type, joystick_device)

        self.lowstate_publisher = self.create_publisher(LowState, TOPIC_LOWSTATE, 10)
        self.secondary_imu_publisher = self.create_publisher(IMUState, TOPIC_SECONDARY_IMU, 10)
        self.bmsstate_publisher = self.create_publisher(BmsState, TOPIC_BMSSTATE, 10)
        self.wireless_controller_publisher = self.create_publisher(WirelessController, TOPIC_WIRELESS_CONTROLLER, 10)
        self.sportmodestate_publisher = self.create_publisher(SportModeState, TOPIC_SPORTMODESTATE, 10)
        self.lowcmd_subscriber = self.create_subscription(LowCmd, TOPIC_LOWCMD, self._lowcmd_callback, 10)

    def _build_sensor_address_map(self) -> dict[str, int]:
        sensor_adr_by_name = {}
        for sensor_idx in range(self.mj_model.nsensor):
            name = self._mj_name(self.mj_model, self._mj_sensor_obj_type(), sensor_idx)
            if name:
                sensor_adr_by_name[name] = int(self.mj_model.sensor_adr[sensor_idx])
        return sensor_adr_by_name

    def _mj_sensor_obj_type(self):
        import mujoco

        return mujoco.mjtObj.mjOBJ_SENSOR

    def _mj_name(self, model, obj_type, idx):
        import mujoco

        return mujoco.mj_id2name(model, obj_type, idx)

    def _lowcmd_callback(self, msg: LowCmd):
        self.latest_low_cmd = msg
        self.latest_mode_pr = int(msg.mode_pr)

    def apply_latest_command(self):
        torques = compute_motor_torques(
            self.latest_low_cmd,
            joint_pos=self.mj_data.sensordata[: self.num_motor],
            joint_vel=self.mj_data.sensordata[self.num_motor : 2 * self.num_motor],
            num_motor=self.num_motor,
        )
        self.mj_data.ctrl[: self.num_motor] = torques

    def publish_step_messages(self):
        snapshot = self.gamepad.snapshot()
        self._publish_lowstate(snapshot)
        self._publish_secondary_imu()
        self._publish_sportmodestate()
        self._publish_wireless_controller(snapshot)
        self._publish_bmsstate()

    def _copy_imu(self, message: IMUState, quat_key: str, gyro_key: str, acc_key: str):
        quat_adr = self.sensor_adr[quat_key]
        gyro_adr = self.sensor_adr[gyro_key]
        acc_adr = self.sensor_adr[acc_key]

        quat = self.mj_data.sensordata[quat_adr : quat_adr + 4]
        gyro = self.mj_data.sensordata[gyro_adr : gyro_adr + 3]
        acc = self.mj_data.sensordata[acc_adr : acc_adr + 3]
        rpy = quat_to_rpy(quat)

        for i in range(4):
            message.quaternion[i] = float(quat[i])
        for i in range(3):
            message.gyroscope[i] = float(gyro[i])
            message.accelerometer[i] = float(acc[i])
            message.rpy[i] = float(rpy[i])
        return message

    def _publish_lowstate(self, snapshot: GamepadSnapshot):
        msg = self.low_state_msg
        msg.mode_pr = self.latest_mode_pr
        msg.mode_machine = self.mode_machine
        msg.tick = int(round(self.mj_data.time / 1e-3))
        self._copy_imu(msg.imu_state, "imu_quat", "imu_gyro", "imu_acc")

        for i in range(NUM_HG_MOTOR_SLOTS):
            motor_state = msg.motor_state[i]
            if i < self.num_motor:
                motor_state.q = float(self.mj_data.sensordata[i])
                motor_state.dq = float(self.mj_data.sensordata[i + self.num_motor])
                motor_state.tau_est = float(self.mj_data.sensordata[i + 2 * self.num_motor])
            else:
                motor_state.q = 0.0
                motor_state.dq = 0.0
                motor_state.tau_est = 0.0

        wireless_remote = encode_wireless_remote(snapshot)
        for i, value in enumerate(wireless_remote):
            msg.wireless_remote[i] = value
        self.lowstate_publisher.publish(msg)

    def _publish_secondary_imu(self):
        msg = self._copy_imu(self.secondary_imu_msg, "secondary_imu_quat", "secondary_imu_gyro", "secondary_imu_acc")
        self.secondary_imu_publisher.publish(msg)

    def _publish_sportmodestate(self):
        msg = self.sport_state_msg
        frame_pos_adr = self.sensor_adr["frame_pos"]
        frame_vel_adr = self.sensor_adr["frame_vel"]
        if frame_pos_adr is None or frame_vel_adr is None:
            if not self.warned_missing_frame_sensor:
                self.get_logger().warn("MuJoCo frame_pos/frame_vel sensors are missing; /sportmodestate will publish zeros.")
                self.warned_missing_frame_sensor = True
            for i in range(3):
                msg.position[i] = 0.0
                msg.velocity[i] = 0.0
        else:
            for i in range(3):
                msg.position[i] = float(self.mj_data.sensordata[frame_pos_adr + i])
                msg.velocity[i] = float(self.mj_data.sensordata[frame_vel_adr + i])
        self.sportmodestate_publisher.publish(msg)

    def _publish_wireless_controller(self, snapshot: GamepadSnapshot):
        msg = populate_wireless_controller_message(snapshot, self.wireless_controller_msg)
        self.wireless_controller_publisher.publish(msg)

    def _publish_bmsstate(self):
        self.bmsstate_publisher.publish(self.bms_state_msg)
