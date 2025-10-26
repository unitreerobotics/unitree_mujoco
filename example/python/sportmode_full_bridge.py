#!/usr/bin/env python3
"""
Unitree SportModeState Full Bridge to ROS2
===========================================

This script publishes the COMPLETE SportModeState to ROS2 in a structured format.

Usage:
    python3 unitree_sportmode_full_bridge.py
    
Then:
    ros2 topic echo /unitree/sportmode/full
    ros2 topic echo /unitree/sportmode/state
    
    
# /unitree/sportmode/full의 데이터 구조
[
    # [0:5] Metadata
    error_code, mode, progress, gait_type, foot_raise_height,
    
    # [5:9] Position & Height
    pos_x, pos_y, pos_z, body_height,
    
    # [9:13] Velocity
    vel_x, vel_y, vel_z, yaw_speed,
    
    # [13:26] IMU (quaternion[4] + gyro[3] + acc[3] + rpy[3])
    quat_w, quat_x, quat_y, quat_z,
    gyro_x, gyro_y, gyro_z,
    acc_x, acc_y, acc_z,
    roll, pitch, yaw,
    
    # [26:30] Foot force
    force_FL, force_FR, force_RL, force_RR,
    
    # [30:42] Foot position body (FL[3], FR[3], RL[3], RR[3])
    FL_x, FL_y, FL_z,
    FR_x, FR_y, FR_z,
    RL_x, RL_y, RL_z,
    RR_x, RR_y, RR_z,
    
    # [42:54] Foot speed body (FL[3], FR[3], RL[3], RR[3])
    FL_vx, FL_vy, FL_vz,
    FR_vx, FR_vy, FR_vz,
    RL_vx, RL_vy, RL_vz,
    RR_vx, RR_vy, RR_vz,
    
    # [54:58] Range obstacle
    range_0, range_1, range_2, range_3
]    
"""

import sys
import signal
import atexit
import numpy as np

import rclpy
from rclpy.node import Node
from std_msgs.msg import Header, Float32MultiArray, UInt8, UInt32, Int16MultiArray, String
from geometry_msgs.msg import Vector3, Quaternion, Pose, Twist
from diagnostic_msgs.msg import KeyValue

from unitree_sdk2py.core.channel import ChannelSubscriber, ChannelFactoryInitialize
from unitree_sdk2py.idl.unitree_go.msg.dds_ import SportModeState_


class SportModeStateFullPublisher(Node):
    """Publish complete SportModeState to ROS2"""
    
    def __init__(self):
        super().__init__('unitree_sportmode_full_bridge')
        
        # Publishers - Structured format
        self.pub_full = self.create_publisher(
            Float32MultiArray, '/unitree/sportmode/full', 10)
        
        self.pub_state_json = self.create_publisher(
            String, '/unitree/sportmode/state_json', 10)
        
        # Publishers - Individual fields (for easy access)
        self.pub_error_code = self.create_publisher(
            UInt32, '/unitree/sportmode/error_code', 10)
        
        self.pub_mode = self.create_publisher(
            UInt8, '/unitree/sportmode/mode', 10)
        
        self.pub_gait_type = self.create_publisher(
            UInt8, '/unitree/sportmode/gait_type', 10)
        
        self.pub_position = self.create_publisher(
            Vector3, '/unitree/sportmode/position', 10)
        
        self.pub_velocity = self.create_publisher(
            Vector3, '/unitree/sportmode/velocity', 10)
        
        self.pub_imu_quat = self.create_publisher(
            Quaternion, '/unitree/sportmode/imu/quaternion', 10)
        
        self.pub_imu_gyro = self.create_publisher(
            Vector3, '/unitree/sportmode/imu/gyroscope', 10)
        
        self.pub_imu_acc = self.create_publisher(
            Vector3, '/unitree/sportmode/imu/accelerometer', 10)
        
        self.pub_foot_force = self.create_publisher(
            Int16MultiArray, '/unitree/sportmode/foot_force', 10)
        
        self.pub_foot_pos_body = self.create_publisher(
            Float32MultiArray, '/unitree/sportmode/foot_position_body', 10)
        
        self.pub_foot_speed_body = self.create_publisher(
            Float32MultiArray, '/unitree/sportmode/foot_speed_body', 10)
        
        # Statistics
        self.msg_count = 0
        self.first_msg = True
        
        # Initialize Unitree DDS
        self._setup_unitree_dds()
        
        # Timer for statistics
        self.stat_timer = self.create_timer(5.0, self._print_statistics)
        
        self.get_logger().info('=' * 70)
        self.get_logger().info('Unitree SportModeState FULL Bridge to ROS2')
        self.get_logger().info('=' * 70)
        self.get_logger().info('Publishing complete SportModeState to:')
        self.get_logger().info('  - /unitree/sportmode/full (Float32MultiArray - all data)')
        self.get_logger().info('  - /unitree/sportmode/state_json (String - JSON format)')
        self.get_logger().info('  - /unitree/sportmode/* (Individual fields)')
        self.get_logger().info('=' * 70)
    
    def _setup_unitree_dds(self):
        """Setup Unitree DDS"""
        if len(sys.argv) < 2:
            ChannelFactoryInitialize(1, "lo")
            self.get_logger().info('Connected to simulation (domain=1)')
        else:
            ChannelFactoryInitialize(0, sys.argv[1])
            self.get_logger().info(f'Connected to real robot: {sys.argv[1]}')
        
        self.sportmode_sub = ChannelSubscriber("rt/sportmodestate", SportModeState_)
        self.sportmode_sub.Init(self._sportmode_callback, 10)
        
        self.get_logger().info('Waiting for SportModeState...')
    
    def _sportmode_callback(self, msg: SportModeState_):
        """Handle SportModeState and publish everything"""
        
        timestamp = self.get_clock().now()
        
        # Print structure on first message
        if self.first_msg:
            self._print_message_structure(msg)
            self.first_msg = False
        
        # Publish full state as structured array
        self._publish_full_state(msg, timestamp)
        
        # Publish as JSON string
        self._publish_json_state(msg, timestamp)
        
        # Publish individual fields
        self._publish_individual_fields(msg, timestamp)
        
        self.msg_count += 1
    
    def _print_message_structure(self, msg: SportModeState_):
        """Print message structure on first receive"""
        self.get_logger().info('')
        self.get_logger().info('📋 SportModeState Structure:')
        self.get_logger().info('-' * 70)
        
        try:
            self.get_logger().info(f'  error_code: {msg.error_code}')
            self.get_logger().info(f'  mode: {msg.mode}')
            self.get_logger().info(f'  progress: {msg.progress}')
            self.get_logger().info(f'  gait_type: {msg.gait_type}')
            self.get_logger().info(f'  foot_raise_height: {msg.foot_raise_height}')
            self.get_logger().info(f'  position: [{msg.position[0]:.3f}, {msg.position[1]:.3f}, {msg.position[2]:.3f}]')
            self.get_logger().info(f'  body_height: {msg.body_height}')
            self.get_logger().info(f'  velocity: [{msg.velocity[0]:.3f}, {msg.velocity[1]:.3f}, {msg.velocity[2]:.3f}]')
            self.get_logger().info(f'  yaw_speed: {msg.yaw_speed}')
            
            if hasattr(msg, 'imu_state') and msg.imu_state:
                self.get_logger().info(f'  imu_state.quaternion: [{msg.imu_state.quaternion[0]:.3f}, ...]')
                self.get_logger().info(f'  imu_state.gyroscope: [{msg.imu_state.gyroscope[0]:.3f}, ...]')
                self.get_logger().info(f'  imu_state.accelerometer: [{msg.imu_state.accelerometer[0]:.3f}, ...]')
            
            self.get_logger().info(f'  foot_force: {list(msg.foot_force[:4])}')
            self.get_logger().info(f'  foot_position_body: [12 elements]')
            self.get_logger().info(f'  foot_speed_body: [12 elements]')
            
        except Exception as e:
            self.get_logger().warn(f'Could not print structure: {e}')
        
        self.get_logger().info('-' * 70)
        self.get_logger().info('')
    
    def _publish_full_state(self, msg: SportModeState_, timestamp):
        """Publish complete state as Float32MultiArray"""
        full_msg = Float32MultiArray()
        
        # Pack all data into one array
        # Layout: [metadata(5), position(3), velocity(4), imu(13), feet(40)]
        data = []
        
        # Metadata (5)
        data.append(float(msg.error_code))
        data.append(float(msg.mode))
        data.append(float(msg.progress))
        data.append(float(msg.gait_type))
        data.append(float(msg.foot_raise_height))
        
        # Position & height (4)
        data.extend([float(x) for x in msg.position])
        data.append(float(msg.body_height))
        
        # Velocity (4: vx, vy, vz, yaw_speed)
        data.extend([float(x) for x in msg.velocity])
        data.append(float(msg.yaw_speed))
        
        # IMU (13: quat[4] + gyro[3] + acc[3] + rpy[3])
        if hasattr(msg, 'imu_state') and msg.imu_state:
            data.extend([float(x) for x in msg.imu_state.quaternion])
            data.extend([float(x) for x in msg.imu_state.gyroscope])
            data.extend([float(x) for x in msg.imu_state.accelerometer])
            if hasattr(msg.imu_state, 'rpy'):
                data.extend([float(x) for x in msg.imu_state.rpy])
            else:
                data.extend([0.0, 0.0, 0.0])
        else:
            data.extend([0.0] * 13)
        
        # Foot force (4)
        data.extend([float(x) for x in msg.foot_force[:4]])
        
        # Foot position body (12)
        data.extend([float(x) for x in msg.foot_position_body[:12]])
        
        # Foot speed body (12)
        data.extend([float(x) for x in msg.foot_speed_body[:12]])
        
        # Range obstacle (4)
        if hasattr(msg, 'range_obstacle'):
            data.extend([float(x) for x in msg.range_obstacle[:4]])
        else:
            data.extend([0.0] * 4)
        
        full_msg.data = data
        self.pub_full.publish(full_msg)
    
    def _publish_json_state(self, msg: SportModeState_, timestamp):
        """Publish as JSON string for easy parsing"""
        import json
        
        state_dict = {
            'timestamp_sec': timestamp.nanoseconds / 1e9,
            'error_code': int(msg.error_code),
            'mode': int(msg.mode),
            'progress': float(msg.progress),
            'gait_type': int(msg.gait_type),
            'foot_raise_height': float(msg.foot_raise_height),
            'position': [float(x) for x in msg.position],
            'body_height': float(msg.body_height),
            'velocity': [float(x) for x in msg.velocity],
            'yaw_speed': float(msg.yaw_speed),
            'foot_force': [int(x) for x in msg.foot_force[:4]],
            'foot_position_body': [float(x) for x in msg.foot_position_body[:12]],
            'foot_speed_body': [float(x) for x in msg.foot_speed_body[:12]],
        }
        
        # Add IMU if available
        if hasattr(msg, 'imu_state') and msg.imu_state:
            state_dict['imu'] = {
                'quaternion': [float(x) for x in msg.imu_state.quaternion],
                'gyroscope': [float(x) for x in msg.imu_state.gyroscope],
                'accelerometer': [float(x) for x in msg.imu_state.accelerometer],
            }
            if hasattr(msg.imu_state, 'rpy'):
                state_dict['imu']['rpy'] = [float(x) for x in msg.imu_state.rpy]
        
        json_msg = String()
        json_msg.data = json.dumps(state_dict, indent=2)
        self.pub_state_json.publish(json_msg)
    
    def _publish_individual_fields(self, msg: SportModeState_, timestamp):
        """Publish individual fields for easy access"""
        
        # Error code
        error_msg = UInt32()
        error_msg.data = msg.error_code
        self.pub_error_code.publish(error_msg)
        
        # Mode
        mode_msg = UInt8()
        mode_msg.data = msg.mode
        self.pub_mode.publish(mode_msg)
        
        # Gait type
        gait_msg = UInt8()
        gait_msg.data = msg.gait_type
        self.pub_gait_type.publish(gait_msg)
        
        # Position
        pos_msg = Vector3()
        pos_msg.x = float(msg.position[0])
        pos_msg.y = float(msg.position[1])
        pos_msg.z = float(msg.position[2])
        self.pub_position.publish(pos_msg)
        
        # Velocity
        vel_msg = Vector3()
        vel_msg.x = float(msg.velocity[0])
        vel_msg.y = float(msg.velocity[1])
        vel_msg.z = float(msg.velocity[2])
        self.pub_velocity.publish(vel_msg)
        
        # IMU
        if hasattr(msg, 'imu_state') and msg.imu_state:
            # Quaternion
            quat_msg = Quaternion()
            quat_msg.w = float(msg.imu_state.quaternion[0])
            quat_msg.x = float(msg.imu_state.quaternion[1])
            quat_msg.y = float(msg.imu_state.quaternion[2])
            quat_msg.z = float(msg.imu_state.quaternion[3])
            self.pub_imu_quat.publish(quat_msg)
            
            # Gyroscope
            gyro_msg = Vector3()
            gyro_msg.x = float(msg.imu_state.gyroscope[0])
            gyro_msg.y = float(msg.imu_state.gyroscope[1])
            gyro_msg.z = float(msg.imu_state.gyroscope[2])
            self.pub_imu_gyro.publish(gyro_msg)
            
            # Accelerometer
            acc_msg = Vector3()
            acc_msg.x = float(msg.imu_state.accelerometer[0])
            acc_msg.y = float(msg.imu_state.accelerometer[1])
            acc_msg.z = float(msg.imu_state.accelerometer[2])
            self.pub_imu_acc.publish(acc_msg)
        
        # Foot force
        force_msg = Int16MultiArray()
        force_msg.data = [int(x) for x in msg.foot_force[:4]]
        self.pub_foot_force.publish(force_msg)
        
        # Foot position body
        foot_pos_msg = Float32MultiArray()
        foot_pos_msg.data = [float(x) for x in msg.foot_position_body[:12]]
        self.pub_foot_pos_body.publish(foot_pos_msg)
        
        # Foot speed body
        foot_speed_msg = Float32MultiArray()
        foot_speed_msg.data = [float(x) for x in msg.foot_speed_body[:12]]
        self.pub_foot_speed_body.publish(foot_speed_msg)
    
    def _print_statistics(self):
        """Print statistics"""
        if self.msg_count > 0:
            self.get_logger().info(
                f'📊 Received {self.msg_count} messages '
                f'(~{self.msg_count/5:.1f} Hz)'
            )
            self.msg_count = 0
        else:
            self.get_logger().warn('⚠️  No messages received')


# =============================================================================
# Cleanup
# =============================================================================

_node_instance = None

def cleanup_handler(signum=None, frame=None):
    global _node_instance
    
    print("\n🧹 Shutting down SportModeState full bridge...")
    
    if _node_instance:
        try:
            _node_instance.destroy_node()
            print("✓ Node destroyed")
        except:
            pass
    
    try:
        if rclpy.ok():
            rclpy.shutdown()
            print("✓ ROS2 shutdown")
    except:
        pass
    
    print("✓ Shutdown complete\n")
    
    if signum is not None:
        sys.exit(0)

signal.signal(signal.SIGINT, cleanup_handler)
signal.signal(signal.SIGTERM, cleanup_handler)
atexit.register(cleanup_handler)


# =============================================================================
# Main
# =============================================================================

def main(args=None):
    global _node_instance
    
    print("\n" + "=" * 70)
    print("Unitree SportModeState FULL Bridge to ROS2")
    print("=" * 70)
    print("Press Ctrl+C to stop")
    print("=" * 70 + "\n")
    
    try:
        rclpy.init(args=args)
        node = SportModeStateFullPublisher()
        _node_instance = node
        rclpy.spin(node)
        return 0
    
    except KeyboardInterrupt:
        print("\n🛑 Interrupted")
        return 0
    
    except Exception as e:
        print(f"\n❌ Error: {e}")
        import traceback
        traceback.print_exc()
        return 1


if __name__ == "__main__":
    exit(main())
