#!/usr/bin/env python3
"""
IsaacLab Policy Runner with ROS2 Diagnostics (with Joint Index Reordering)
==========================================================================

This script executes a trained IsaacLab policy on the Unitree Go2 robot using MuJoCo simulation
and publishes detailed observation data to ROS2 topics for analysis with PlotJuggler.

Key Features:
- ONNX-based policy execution
- ROS2 publishers for all observation components
- Real-time diagnostics data streaming
- Proper observation scaling matching training conditions
- Joint index reordering between Unitree SDK/MuJoCo and IsaacLab

Joint Index Conversion:
- Unitree/MuJoCo order: FR_hip, FR_thigh, FR_calf, FL_hip, FL_thigh, FL_calf, 
                        RR_hip, RR_thigh, RR_calf, RL_hip, RL_thigh, RL_calf
- IsaacLab order: FL_hip, FR_hip, RL_hip, RR_hip,
                  FL_thigh, FR_thigh, RL_thigh, RR_thigh,
                  FL_calf, FR_calf, RL_calf, RR_calf

Author: Generated for IsaacLab policy integration with ROS2 diagnostics
"""

import time
import sys
import numpy as np
import onnxruntime as ort
from typing import Optional, Tuple

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Float32, Header
from geometry_msgs.msg import Vector3Stamped, TwistStamped
from sensor_msgs.msg import JointState

from unitree_sdk2py.core.channel import ChannelPublisher, ChannelSubscriber, ChannelFactoryInitialize
from unitree_sdk2py.idl.default import unitree_go_msg_dds__LowCmd_, unitree_go_msg_dds__LowState_
from unitree_sdk2py.idl.unitree_go.msg.dds_ import LowCmd_, LowState_, SportModeState_
from unitree_sdk2py.utils.crc import CRC

# =============================================================================
# Joint Index Reordering
# =============================================================================

# Unitree SDK/MuJoCo -> IsaacLab 변환
# IsaacLab[i] = Unitree[UNITREE_TO_ISAACLAB[i]]
UNITREE_TO_ISAACLAB = np.array([
    3, 0, 9, 6,   # FL_hip, FR_hip, RL_hip, RR_hip
    4, 1, 10, 7,  # FL_thigh, FR_thigh, RL_thigh, RR_thigh
    5, 2, 11, 8   # FL_calf, FR_calf, RL_calf, RR_calf
], dtype=np.int32)

# IsaacLab -> Unitree SDK/MuJoCo 변환
# Unitree[i] = IsaacLab[ISAACLAB_TO_UNITREE[i]]
ISAACLAB_TO_UNITREE = np.array([
    1, 5, 9,   # FR_hip, FR_thigh, FR_calf
    0, 4, 8,   # FL_hip, FL_thigh, FL_calf
    3, 7, 11,  # RR_hip, RR_thigh, RR_calf
    2, 6, 10   # RL_hip, RL_thigh, RL_calf
], dtype=np.int32)

# =============================================================================
# Configuration Parameters
# =============================================================================

class PolicyConfig:
    """Configuration parameters matching IsaacLab training setup"""
    
    CONTROL_DT = 0.02  # 50Hz control frequency
    
    NOISE_LEVELS = {
        'base_lin_vel': (-0.1, 0.1),
        'base_ang_vel': (-0.2, 0.2),
        'projected_gravity': (-0.05, 0.05),
        'joint_pos': (-0.01, 0.01),
        'joint_vel': (-1.5, 1.5),
    }
    
    ACTION_SCALE = 0.25
    
    # DEFAULT_JOINT_POS는 Unitree/MuJoCo 순서로 정의됨. IsaacLab 순서로 변환 필요.
    DEFAULT_JOINT_POS = np.array([
        -0.1, 0.8, -1.5,   # FR_hip, FR_thigh, FR_calf
        0.1, 0.8, -1.5,    # FL_hip, FL_thigh, FL_calf
        -0.1, 1.0, -1.5,   # RR_hip, RR_thigh, RR_calf
        0.1, 1.0, -1.5,    # RL_hip, RL_thigh, RL_calf
    ], dtype=np.float32)

    # Unitree/MuJoCo 순서 → IsaacLab 순서로 변환
    DEFAULT_JOINT_POS = DEFAULT_JOINT_POS[UNITREE_TO_ISAACLAB]
    
    KP = 25.0
    KD = 0.5
    
    OBS_CLIP = 100.0
    ACTION_CLIP = 23.5  # effort limit

# =============================================================================
# ROS2 Diagnostics Node
# =============================================================================

class Go2DiagnosticsPublisher(Node):
    """ROS2 node for publishing Go2 diagnostics data"""
    
    def __init__(self):
        super().__init__('go2_diagnostics_publisher')
        
        # Publishers for observation components
        self.pub_base_lin_vel = self.create_publisher(
            Vector3Stamped, '/go2/obs/base_lin_vel', 10)
        self.pub_base_ang_vel = self.create_publisher(
            Vector3Stamped, '/go2/obs/base_ang_vel', 10)
        self.pub_projected_gravity = self.create_publisher(
            Vector3Stamped, '/go2/obs/projected_gravity', 10)
        self.pub_velocity_commands = self.create_publisher(
            Vector3Stamped, '/go2/obs/velocity_commands', 10)
        
        # Joint states publisher (IsaacLab order for visualization)
        self.pub_joint_states = self.create_publisher(
            JointState, '/go2/obs/joint_states', 10)
        
        # Actions publisher
        self.pub_actions = self.create_publisher(
            Float32MultiArray, '/go2/actions/raw', 10)
        self.pub_actions_scaled = self.create_publisher(
            Float32MultiArray, '/go2/actions/scaled', 10)
        self.pub_target_positions = self.create_publisher(
            Float32MultiArray, '/go2/actions/target_positions', 10)
        self.pub_computed_torques = self.create_publisher(
            Float32MultiArray, '/go2/actions/computed_torques', 10)
        
        # Full observation vector
        self.pub_full_obs = self.create_publisher(
            Float32MultiArray, '/go2/obs/full_vector', 10)
        
        # Policy diagnostics
        self.pub_inference_time = self.create_publisher(
            Float32, '/go2/diagnostics/inference_time_ms', 10)
        self.pub_loop_time = self.create_publisher(
            Float32, '/go2/diagnostics/loop_time_ms', 10)
        
        # Heading command diagnostics
        self.pub_current_heading = self.create_publisher(
            Float32, '/go2/diagnostics/current_heading_rad', 10)
        self.pub_target_heading = self.create_publisher(
            Float32, '/go2/diagnostics/target_heading_rad', 10)
        self.pub_heading_error = self.create_publisher(
            Float32, '/go2/diagnostics/heading_error_rad', 10)
        
        # Joint names (IsaacLab order for JointState message)
        self.joint_names = [
            'FL_hip', 'FL_thigh', 'FL_calf',
            'FR_hip', 'FR_thigh', 'FR_calf',
            'RL_hip', 'RL_thigh', 'RL_calf',
            'RR_hip', 'RR_thigh', 'RR_calf'
        ]
        
        self.get_logger().info('Go2 Diagnostics Publisher initialized')
    
    def publish_observations(self, obs_dict: dict, timestamp: float):
        """Publish all observation components"""
        
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = 'base_link'
        
        # Base linear velocity
        msg = Vector3Stamped()
        msg.header = header
        msg.vector.x = float(obs_dict['base_lin_vel'][0])
        msg.vector.y = float(obs_dict['base_lin_vel'][1])
        msg.vector.z = float(obs_dict['base_lin_vel'][2])
        self.pub_base_lin_vel.publish(msg)
        
        # Base angular velocity
        msg = Vector3Stamped()
        msg.header = header
        msg.vector.x = float(obs_dict['base_ang_vel'][0])
        msg.vector.y = float(obs_dict['base_ang_vel'][1])
        msg.vector.z = float(obs_dict['base_ang_vel'][2])
        self.pub_base_ang_vel.publish(msg)
        
        # Projected gravity
        msg = Vector3Stamped()
        msg.header = header
        msg.vector.x = float(obs_dict['projected_gravity'][0])
        msg.vector.y = float(obs_dict['projected_gravity'][1])
        msg.vector.z = float(obs_dict['projected_gravity'][2])
        self.pub_projected_gravity.publish(msg)
        
        # Velocity commands
        msg = Vector3Stamped()
        msg.header = header
        msg.vector.x = float(obs_dict['velocity_commands'][0])
        msg.vector.y = float(obs_dict['velocity_commands'][1])
        msg.vector.z = float(obs_dict['velocity_commands'][2])
        self.pub_velocity_commands.publish(msg)
        
        # Joint states (already in IsaacLab order from obs_dict)
        joint_msg = JointState()
        joint_msg.header = header
        joint_msg.name = self.joint_names
        joint_msg.position = [float(p) for p in obs_dict['joint_pos']]
        joint_msg.velocity = [float(v) for v in obs_dict['joint_vel']]
        self.pub_joint_states.publish(joint_msg)
        
        # Full observation vector
        full_obs_msg = Float32MultiArray()
        full_obs_msg.data = [float(x) for x in obs_dict['full_obs']]
        self.pub_full_obs.publish(full_obs_msg)
    
    def publish_actions(self, actions_raw: np.ndarray, actions_scaled: np.ndarray, 
                       target_positions: np.ndarray, computed_torques: np.ndarray = None):
        """Publish action data (all in IsaacLab order for consistency with observations)"""
        
        # Raw actions from policy (IsaacLab order)
        msg = Float32MultiArray()
        msg.data = [float(x) for x in actions_raw]
        self.pub_actions.publish(msg)
        
        # Scaled actions (IsaacLab order)
        msg = Float32MultiArray()
        msg.data = [float(x) for x in actions_scaled]
        self.pub_actions_scaled.publish(msg)
        
        # Target joint positions (IsaacLab order)
        msg = Float32MultiArray()
        msg.data = [float(x) for x in target_positions]
        self.pub_target_positions.publish(msg)
        
        # Computed torques (IsaacLab order)
        if computed_torques is not None:
            msg = Float32MultiArray()
            msg.data = [float(x) for x in computed_torques]
            self.pub_computed_torques.publish(msg)
    
    def publish_diagnostics(self, inference_time_ms: float, loop_time_ms: float, 
                           current_heading: float = 0.0, target_heading: float = 0.0, 
                           heading_error: float = 0.0):
        """Publish timing and heading diagnostics"""
        
        msg = Float32()
        msg.data = inference_time_ms
        self.pub_inference_time.publish(msg)
        
        msg = Float32()
        msg.data = loop_time_ms
        self.pub_loop_time.publish(msg)
        
        # Heading diagnostics
        msg = Float32()
        msg.data = current_heading
        self.pub_current_heading.publish(msg)
        
        msg = Float32()
        msg.data = target_heading
        self.pub_target_heading.publish(msg)
        
        msg = Float32()
        msg.data = heading_error
        self.pub_heading_error.publish(msg)

# =============================================================================
# Observation Processing
# =============================================================================

class ObservationProcessor:
    """Processes raw sensor data into policy observations with joint reordering"""
    
    def __init__(self, config: PolicyConfig):
        self.config = config
        # last_actions는 IsaacLab 순서로 저장됨
        self.last_actions = np.zeros(12, dtype=np.float32)
        
        # Store latest velocity from SportModeState
        self.latest_base_velocity = np.zeros(3, dtype=np.float32)
        
        # Velocity command configuration (matching IsaacLab pattern)
        self.heading_command = True  # 학습 시 설정과 동일하게 (absolute heading mode)
        self.heading_control_stiffness = 0.5  # IsaacLab default value
        
        # Input velocity commands [vx, vy, yaw] - what user provides
        self.velocity_commands = np.zeros(3, dtype=np.float32)  # [vx, vy, yaw_target or yaw_rate]
        
        # Processed velocity commands [vx, vy, wz] - what robot actually uses and goes to policy
        self.vel_command_b = np.zeros(3, dtype=np.float32)  # processed velocity commands
        
        # Heading control state (for absolute heading mode)
        self.heading_target = 0.0      # target heading (radians)
    
    def update_base_velocity(self, velocity):
        """Update base velocity from SportModeState"""
        self.latest_base_velocity = np.array(velocity, dtype=np.float32)
        
    def process(self, lowstate: LowState_, add_noise: bool = True) -> Tuple[np.ndarray, dict]:
        """Process raw sensor data and return observation + components dict
        
        CRITICAL: 
        - MuJoCo의 joint data를 받아서 IsaacLab 순서로 변환
        - Policy에 IsaacLab 순서의 observation 입력
        """
        
        # Use base velocity from bridge (SportModeState)
        base_lin_vel = self.latest_base_velocity.copy()
        
        # Get angular velocity from IMU
        base_ang_vel = np.array(lowstate.imu_state.gyroscope, dtype=np.float32)
        
        # Projected gravity and current orientation
        quat = np.array(lowstate.imu_state.quaternion, dtype=np.float32)
        projected_gravity = self._compute_projected_gravity(quat)
        
        # Update processed velocity commands based on heading mode
        self._update_vel_command_b(quat)
        
        # ===== CRITICAL: Joint Index Reordering =====
        # MuJoCo에서 받은 joint 데이터 (Unitree 순서)
        joint_pos_unitree = np.array([ms.q for ms in lowstate.motor_state[:12]], dtype=np.float32)
        joint_vel_unitree = np.array([ms.dq for ms in lowstate.motor_state[:12]], dtype=np.float32)
        
        # Unitree 순서 -> IsaacLab 순서로 변환
        joint_pos = joint_pos_unitree[UNITREE_TO_ISAACLAB]
        joint_vel = joint_vel_unitree[UNITREE_TO_ISAACLAB]
        
        # Relative joint positions (IsaacLab 순서)
        joint_pos_rel = joint_pos - self.config.DEFAULT_JOINT_POS
        
        # Store clean observations before noise (모두 IsaacLab 순서)
        obs_clean = {
            'base_lin_vel': base_lin_vel.copy(),
            'base_ang_vel': base_ang_vel.copy(),
            'projected_gravity': projected_gravity.copy(),
            'velocity_commands': self.vel_command_b.copy(),
            'joint_pos': joint_pos_rel.copy(),
            'joint_vel': joint_vel.copy(),
            'last_actions': self.last_actions.copy()
        }
        
        # Add noise if enabled
        if add_noise:
            base_lin_vel += self._add_noise(base_lin_vel, 'base_lin_vel')
            base_ang_vel += self._add_noise(base_ang_vel, 'base_ang_vel')
            projected_gravity += self._add_noise(projected_gravity, 'projected_gravity')
            joint_pos_rel += self._add_noise(joint_pos_rel, 'joint_pos')
            joint_vel += self._add_noise(joint_vel, 'joint_vel')
        
        # Concatenate observation (모두 IsaacLab 순서)
        obs = np.concatenate([
            base_lin_vel,
            base_ang_vel,
            projected_gravity,
            self.vel_command_b,
            joint_pos_rel,
            joint_vel,
            self.last_actions
        ]).astype(np.float32)
        
        # Clip observations
        obs = np.clip(obs, -self.config.OBS_CLIP, self.config.OBS_CLIP)
        
        # Update components dict with noisy values
        obs_clean['full_obs'] = obs
        
        return obs, obs_clean
    
    def update_last_actions(self, actions: np.ndarray):
        """Update the last actions (IsaacLab 순서로 저장)"""
        self.last_actions = actions.copy()
    
    def set_velocity_commands(self, vx: float, vy: float, yaw: float):
        """Set velocity commands [vx, vy, yaw]
        
        Args:
            vx: Forward velocity command (m/s)
            vy: Lateral velocity command (m/s) 
            yaw: Target heading (rad) if heading_command=True,
                 or angular velocity (rad/s) if heading_command=False
        """
        # Store input velocity commands
        self.velocity_commands = np.array([vx, vy, yaw], dtype=np.float32)
        
        # Copy linear velocities to processed commands
        self.vel_command_b[0] = vx  # vx always direct
        self.vel_command_b[1] = vy  # vy always direct
        
        if self.heading_command:
            # Store target heading for absolute heading control
            self.heading_target = yaw
            # Angular velocity will be computed in _update_vel_command_b()
        else:
            # Direct angular velocity command (relative mode)
            self.vel_command_b[2] = yaw
    
    def _update_vel_command_b(self, quat: np.ndarray):
        """Update vel_command_b based on heading mode (matching IsaacLab pattern exactly)
        
        Following IsaacLab code (simplified for single environment):
        if self.cfg.heading_command:
            heading_error = math_utils.wrap_to_pi(self.heading_target - self.robot.data.heading_w)
            self.vel_command_b[2] = torch.clip(
                self.cfg.heading_control_stiffness * heading_error,
                min=self.cfg.ranges.ang_vel_z[0],
                max=self.cfg.ranges.ang_vel_z[1],
            )
        """
        if self.heading_command:
            # Compute current heading from quaternion (equivalent to self.robot.data.heading_w)
            current_heading = self._compute_heading_from_quat(quat)
            
            # Compute heading error (target - current, wrapped to [-pi, pi])
            heading_error = self._wrap_to_pi(self.heading_target - current_heading)
            
            # Compute angular velocity with stiffness control and clipping
            # (equivalent to IsaacLab's torch.clip with cfg.ranges.ang_vel_z)
            angular_velocity = self.heading_control_stiffness * heading_error
            ang_vel_limit = 1.0  # equivalent to cfg.ranges.ang_vel_z
            self.vel_command_b[2] = np.clip(angular_velocity, -ang_vel_limit, ang_vel_limit)
        
        # If heading_command=False (relative mode), vel_command_b[2] is already set 
        # directly in set_velocity_commands() as angular velocity command
    
    def _compute_projected_gravity(self, quat: np.ndarray) -> np.ndarray:
        """Compute normalized gravity vector in body frame (matching Isaac Lab)
        
        Isaac Lab process:
        1. Get gravity from simulator: [0, 0, -9.81]
        2. Normalize to unit vector: [0, 0, -1]
        3. Transform to body frame using quaternion inverse
        4. Result: normalized direction vector in body frame
        
        Args:
            quat: Quaternion [w, x, y, z]
            
        Returns:
            Normalized gravity vector in body frame [-1, 1]
        """
        w, x, y, z = quat
        
        # World-to-body rotation matrix (transpose of body-to-world)
        R_T = np.array([
            [1-2*(y*y+z*z), 2*(x*y+w*z), 2*(x*z-w*y)],
            [2*(x*y-w*z), 1-2*(x*x+z*z), 2*(y*z+w*x)],
            [2*(x*z+w*y), 2*(y*z-w*x), 1-2*(x*x+y*y)]
        ], dtype=np.float32)
        
        # Gravity in world frame (normalized direction vector)
        # Isaac Lab normalizes: gravity_dir = normalize([0, 0, -9.81]) = [0, 0, -1]
        gravity_world_normalized = np.array([0.0, 0.0, -1.0], dtype=np.float32)
        
        # Transform to body frame
        gravity_body = R_T @ gravity_world_normalized
        
        # Ensure it's normalized (should already be unit vector from rotation)
        gravity_body_norm = np.linalg.norm(gravity_body)
        if gravity_body_norm > 0:
            gravity_body = gravity_body / gravity_body_norm
        
        return gravity_body
    
    def _compute_heading_from_quat(self, quat: np.ndarray) -> float:
        """Compute heading (yaw) angle from quaternion
        
        Args:
            quat: Quaternion [w, x, y, z]
            
        Returns:
            Heading angle in radians [-pi, pi]
        """
        w, x, y, z = quat
        
        # Convert quaternion to yaw angle
        # yaw = atan2(2*(w*z + x*y), 1 - 2*(y*y + z*z))
        yaw = np.arctan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
        
        return float(yaw)
    
    def _wrap_to_pi(self, angle: float) -> float:
        """Wrap angle to [-pi, pi] range (matching IsaacLab math_utils.wrap_to_pi)"""
        return np.arctan2(np.sin(angle), np.cos(angle))
    
    def _add_noise(self, data: np.ndarray, noise_type: str) -> np.ndarray:
        """Add uniform noise"""
        if noise_type not in self.config.NOISE_LEVELS:
            return np.zeros_like(data)
            
        n_min, n_max = self.config.NOISE_LEVELS[noise_type]
        noise = np.random.uniform(n_min, n_max, size=data.shape).astype(np.float32)
        return noise

# =============================================================================
# Main Policy Runner with ROS2
# =============================================================================

class IsaacLabPolicyRunnerROS2:
    """Policy runner with ROS2 diagnostics publishing and joint reordering"""
    
    def __init__(self, policy_path: str, config: PolicyConfig, ros_node: Go2DiagnosticsPublisher):
        self.config = config
        self.crc = CRC()
        self.ros_node = ros_node
        
        # Load ONNX policy
        print(f"Loading ONNX policy from: {policy_path}")
        self.ort_session = ort.InferenceSession(policy_path)
        
        self.input_name = self.ort_session.get_inputs()[0].name
        self.output_name = self.ort_session.get_outputs()[0].name
        
        print(f"Policy input: {self.input_name}")
        print(f"Policy output: {self.output_name}")
        
        # Initialize observation processor (no bridge dependency)
        self.obs_processor = ObservationProcessor(config)
        
        # Communication setup
        self.latest_lowstate: Optional[LowState_] = None
        self.latest_sportmode: Optional[SportModeState_] = None
        self._setup_communication()
        
    def _setup_communication(self):
        """Setup Unitree SDK communication"""
        if len(sys.argv) < 2:
            ChannelFactoryInitialize(1, "lo")
        else:
            ChannelFactoryInitialize(0, sys.argv[1])
        
        self.state_sub = ChannelSubscriber("rt/lowstate", LowState_)
        self.state_sub.Init(self._lowstate_handler, 10)
        
        # Add SportModeState subscriber for velocity data
        self.sportmode_sub = ChannelSubscriber("rt/sportmodestate", SportModeState_)
        self.sportmode_sub.Init(self._sportmode_handler, 10)
        
        self.cmd_pub = ChannelPublisher("rt/lowcmd", LowCmd_)
        self.cmd_pub.Init()
        
        print("Communication channels initialized (LowState + SportModeState)")
    
    def _lowstate_handler(self, msg: LowState_):
        """Handle incoming robot state"""
        self.latest_lowstate = msg
    
    def _sportmode_handler(self, msg: SportModeState_):
        """Handle incoming SportModeState"""
        self.latest_sportmode = msg
        # Update observation processor with velocity data
        velocity = [float(msg.velocity[0]), float(msg.velocity[1]), float(msg.velocity[2])]
        self.obs_processor.update_base_velocity(velocity)
    
    def wait_for_robot_state(self, timeout: float = 5.0):
        """Wait for first robot state"""
        print("Waiting for robot state...")
        start_time = time.time()
        
        while self.latest_lowstate is None:
            if time.time() - start_time > timeout:
                raise TimeoutError("Failed to receive robot state")
            time.sleep(0.001)
        
        print("Robot state received!")
    
    def execute_policy(self, obs: np.ndarray) -> Tuple[np.ndarray, float]:
        """Execute policy and return actions + inference time
        
        Returns:
            actions_raw: Policy output in IsaacLab order
            inference_time: Inference time in milliseconds
        """
        inference_start = time.perf_counter()
        
        obs_batch = obs.reshape(1, -1)
        outputs = self.ort_session.run([self.output_name], {self.input_name: obs_batch})
        actions_raw = outputs[0][0]  # IsaacLab 순서의 action
        
        inference_time = (time.perf_counter() - inference_start) * 1000  # ms
        
        return actions_raw.astype(np.float32), inference_time
    
    
    def _compute_torques(self, target_positions_unitree: np.ndarray, 
                        current_positions_unitree: np.ndarray, 
                        current_velocities_unitree: np.ndarray) -> np.ndarray:
        """Compute torques using PD controller (matching IsaacLab pipeline)
        
        Args:
            target_positions_unitree: Target joint positions in Unitree order
            current_positions_unitree: Current joint positions in Unitree order  
            current_velocities_unitree: Current joint velocities in Unitree order
            
        Returns:
            computed_torques: Computed torques in Unitree order
        """
        # Position and velocity errors
        position_error = target_positions_unitree - current_positions_unitree
        velocity_error = np.zeros_like(current_velocities_unitree)  # Target velocity = 0
        velocity_error = velocity_error - current_velocities_unitree
        
        # PD controller (matching IsaacLab DCMotor parameters)
        kp = self.config.KP  # 25.0
        kd = self.config.KD  # 0.5
        
        computed_torques = kp * position_error + kd * velocity_error
        
        # Apply torque limits (matching IsaacLab DCMotor clipping)
        torque_limit = self.config.ACTION_CLIP  # 23.5 N⋅m
        computed_torques = np.clip(computed_torques, -torque_limit, torque_limit)
        
        return computed_torques
    
    def create_torque_command(self, target_positions_unitree: np.ndarray, 
                             current_positions_unitree: np.ndarray,
                             current_velocities_unitree: np.ndarray) -> unitree_go_msg_dds__LowCmd_:
        """Create torque-based motor command (matching IsaacLab behavior)
        
        Args:
            target_positions_unitree: Target positions in Unitree/MuJoCo order
            current_positions_unitree: Current positions in Unitree/MuJoCo order
            current_velocities_unitree: Current velocities in Unitree/MuJoCo order
        """
        # Compute torques using PD controller
        torques = self._compute_torques(target_positions_unitree, 
                                       current_positions_unitree, 
                                       current_velocities_unitree)
        
        cmd = unitree_go_msg_dds__LowCmd_()
        
        cmd.head[0] = 0xFE
        cmd.head[1] = 0xEF
        cmd.level_flag = 0xFF
        cmd.gpio = 0
        
        # Set torque commands for leg joints (0-11)
        for i in range(12):
            cmd.motor_cmd[i].mode = 0x01  # Position mode but we'll use torque
            cmd.motor_cmd[i].q = 0.0      # Position target (not used in torque mode)
            cmd.motor_cmd[i].kp = 0.0     # Zero position gain (pure torque control)
            cmd.motor_cmd[i].kd = 0.0     # Zero damping gain (pure torque control)
            cmd.motor_cmd[i].dq = 0.0     # Target velocity
            cmd.motor_cmd[i].tau = float(torques[i])  # Computed torque command
        
        # Set other joints to zero
        for i in range(12, 20):
            cmd.motor_cmd[i].mode = 0x01
            cmd.motor_cmd[i].q = 0.0
            cmd.motor_cmd[i].kp = 0.0
            cmd.motor_cmd[i].kd = 0.0
            cmd.motor_cmd[i].dq = 0.0
            cmd.motor_cmd[i].tau = 0.0
        
        cmd.crc = self.crc.Crc(cmd)
        
        return cmd
    
    def _create_zero_command(self) -> unitree_go_msg_dds__LowCmd_:
        """Create zero torque command to safely stop the robot"""
        cmd = unitree_go_msg_dds__LowCmd_()
        
        cmd.head[0] = 0xFE
        cmd.head[1] = 0xEF
        cmd.level_flag = 0xFF
        cmd.gpio = 0
        
        # Set all motors to zero torque mode
        for i in range(20):
            cmd.motor_cmd[i].mode = 0x01  # Keep position mode but with zero gains
            cmd.motor_cmd[i].q = 0.0      # Position doesn't matter with zero gains
            cmd.motor_cmd[i].kp = 0.0     # Zero position gain = no position control
            cmd.motor_cmd[i].kd = 0.0     # Zero damping gain = no velocity control  
            cmd.motor_cmd[i].dq = 0.0     # Zero target velocity
            cmd.motor_cmd[i].tau = 0.0    # Zero torque command
        
        cmd.crc = self.crc.Crc(cmd)
        
        return cmd
    
    def run(self, duration: Optional[float] = None, enable_noise: bool = True):
        """Main execution loop with ROS2 publishing and joint reordering"""
        print("Starting policy execution with ROS2 diagnostics...")
        print(f"Control frequency: {1.0/self.config.CONTROL_DT:.1f} Hz")
        print(f"Observation noise: {'Enabled' if enable_noise else 'Disabled'}")
        print("Joint index reordering: Enabled (Unitree/MuJoCo <-> IsaacLab)")
        
        self.wait_for_robot_state()
        
        # Set initial velocity commands (all zeros initially)
        # For heading mode: (vx=0.0 m/s, vy=0.0, target_heading=0.0 rad)
        # For relative mode: (vx=0.0 m/s, vy=0.0, wz=0.0 rad/s)
        self.obs_processor.set_velocity_commands(0.0, 0.0, -1.0)
        
        start_time = time.time()
        step_count = 0
        
        try:
            while rclpy.ok():
                loop_start = time.perf_counter()
                
                # Check duration
                if duration and (time.time() - start_time) > duration:
                    break
                
                # Get robot state
                if self.latest_lowstate is None:
                    time.sleep(0.001)
                    continue
                
                # Process observations (MuJoCo -> IsaacLab 순서 변환)
                obs, obs_components = self.obs_processor.process(
                    self.latest_lowstate, add_noise=enable_noise)
                
                # Execute policy (IsaacLab 순서 입력 -> IsaacLab 순서 출력)
                actions_raw_isaaclab, inference_time = self.execute_policy(obs)
                
                # Scale actions (IsaacLab 순서)
                actions_scaled_isaaclab = actions_raw_isaaclab * self.config.ACTION_SCALE
                target_positions_isaaclab = actions_scaled_isaaclab + self.config.DEFAULT_JOINT_POS
                
                # ===== CRITICAL: Convert actions to Unitree order for robot control =====
                actions_raw_unitree = actions_raw_isaaclab[ISAACLAB_TO_UNITREE]
                actions_scaled_unitree = actions_scaled_isaaclab[ISAACLAB_TO_UNITREE]
                target_positions_unitree = target_positions_isaaclab[ISAACLAB_TO_UNITREE]
                
                # Update last actions (IsaacLab 순서로 저장)
                self.obs_processor.update_last_actions(actions_raw_isaaclab)
                
                # Get heading diagnostics for ROS2 publishing
                current_heading = 0.0
                target_heading = 0.0
                heading_error = 0.0
                
                if self.obs_processor.heading_command:
                    quat = np.array(self.latest_lowstate.imu_state.quaternion, dtype=np.float32)
                    current_heading = self.obs_processor._compute_heading_from_quat(quat)
                    target_heading = self.obs_processor.heading_target
                    heading_error = self.obs_processor._wrap_to_pi(target_heading - current_heading)
                
                # Get current joint states (Unitree 순서)
                current_positions_unitree = np.array([ms.q for ms in self.latest_lowstate.motor_state[:12]], dtype=np.float32)
                current_velocities_unitree = np.array([ms.dq for ms in self.latest_lowstate.motor_state[:12]], dtype=np.float32)
                
                # Compute torques for diagnostics (Unitree 순서)
                computed_torques_unitree = self._compute_torques(target_positions_unitree, 
                                                               current_positions_unitree, 
                                                               current_velocities_unitree)
                
                # Convert torques to IsaacLab order for ROS2 publishing
                computed_torques_isaaclab = computed_torques_unitree[UNITREE_TO_ISAACLAB]
                
                # Publish ROS2 diagnostics (obs_components는 IsaacLab 순서)
                current_time = time.time() - start_time
                self.ros_node.publish_observations(obs_components, current_time)
                # actions는 IsaacLab 순서로 publish (obs의 previous_actions와 일치)
                self.ros_node.publish_actions(actions_raw_isaaclab, actions_scaled_isaaclab, 
                                              target_positions_isaaclab, computed_torques_isaaclab)
                
                # Spin ROS2 node to process callbacks and publish messages
                rclpy.spin_once(self.ros_node, timeout_sec=0.0)
                
                # Send torque-based motor command (Unitree 순서)
                cmd = self.create_torque_command(target_positions_unitree, 
                                               current_positions_unitree, 
                                               current_velocities_unitree)
                self.cmd_pub.Write(cmd)
                
                # Calculate loop time
                loop_time = (time.perf_counter() - loop_start) * 1000  # ms
                self.ros_node.publish_diagnostics(inference_time, loop_time, 
                                                 current_heading, target_heading, heading_error)
                
                # Additional spin to ensure diagnostics are published
                rclpy.spin_once(self.ros_node, timeout_sec=0.0)
                
                # Timing control
                elapsed = time.perf_counter() - loop_start
                sleep_time = self.config.CONTROL_DT - elapsed
                
                if sleep_time > 0:
                    time.sleep(sleep_time)
                elif sleep_time < -0.001:
                    self.ros_node.get_logger().warning(
                        f'Loop running {-sleep_time*1000:.1f}ms behind')
                
                step_count += 1
                
                # Status update
                if step_count % (int(2.0 / self.config.CONTROL_DT)) == 0:
                    self.ros_node.get_logger().info(
                        f'Step {step_count}, Runtime: {current_time:.1f}s')
        
        except KeyboardInterrupt:
            print("\nStopping...")
        
        finally:
            # Stop robot with zero torque command
            zero_cmd = self._create_zero_command()
            for _ in range(10):
                self.cmd_pub.Write(zero_cmd)
                time.sleep(0.01)
            
            print(f"Completed. Total steps: {step_count}")

# =============================================================================
# Main Entry Point
# =============================================================================

def main(args=None):
    """Main function"""
    print("=" * 60)
    print("IsaacLab Policy Runner with ROS2 Diagnostics")
    print("(with Joint Index Reordering)")
    print("=" * 60)
    
    # Initialize ROS2
    rclpy.init(args=args)
    ros_node = Go2DiagnosticsPublisher()
    
    # Configuration
    config = PolicyConfig()
    policy_path = "/home/jeonchanwook/unitree_mujoco/example/python/exported/policy.onnx"
    
    print("\nWARNING: Ensure robot area is clear!")
    print("ROS2 topics available for PlotJuggler:")
    print("  Policy Observations:")
    print("    - /go2/obs/base_lin_vel (from SportModeState)")
    print("    - /go2/obs/base_ang_vel")
    print("    - /go2/obs/projected_gravity")
    print("    - /go2/obs/velocity_commands")
    print("    - /go2/obs/joint_states (IsaacLab order)")
    print("  Policy Actions:")
    print("    - /go2/actions/raw (IsaacLab order)")
    print("    - /go2/actions/scaled (IsaacLab order)")
    print("    - /go2/actions/target_positions (IsaacLab order)")
    print("    - /go2/actions/computed_torques (IsaacLab order)")
    print("  Diagnostics:")
    print("    - /go2/diagnostics/inference_time_ms")
    print("    - /go2/diagnostics/loop_time_ms")
    print("\nUse: ros2 run plotjuggler plotjuggler")
    print("Note: Run ros2_bridge_sportmodestate.py separately if you need ROS2 bridge topics")
    input("\nPress Enter to start...")
    
    try:
        runner = IsaacLabPolicyRunnerROS2(policy_path, config, ros_node)
        runner.run(duration=30.0, enable_noise=True)
        
    except Exception as e:
        ros_node.get_logger().error(f'Error: {e}')
        return 1
    
    finally:
        ros_node.destroy_node()
        rclpy.shutdown()
    
    return 0

if __name__ == "__main__":
    exit(main())
