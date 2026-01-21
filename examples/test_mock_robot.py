#!/usr/bin/env python3
"""
Mock ROS2 Robot Node for Testing
Simulates a 6-axis robot publishing joint_states and cartesian_pose
and responding to commands. Use this to test the HMI without Isaac Sim.

Usage:
    source /opt/ros/humble/setup.bash
    python test_mock_robot.py --namespace robot1
"""

import argparse
import math
import time
import threading
from typing import List

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header


class MockRobotNode(Node):
    """Simulates a 6-axis robot with ROS2 topics"""
    
    def __init__(self, namespace: str, num_joints: int = 6):
        super().__init__(f'{namespace}_mock_robot')
        
        self.namespace = namespace
        self.num_joints = num_joints
        
        # Current state
        self.joint_positions = [0.0] * num_joints
        self.joint_velocities = [0.0] * num_joints
        self.joint_efforts = [0.0] * num_joints
        
        # Target positions (for simulating motion)
        self.target_positions = [0.0] * num_joints
        
        # Cartesian pose (simulated)
        self.position = {'x': 0.5, 'y': 0.0, 'z': 0.5}
        self.orientation = {'w': 1.0, 'x': 0.0, 'y': 0.0, 'z': 0.0}
        
        # Joint names (matches Isaac Sim convention)
        self.joint_names = [f'joint{i+1}' for i in range(num_joints)]
        
        # QoS profiles
        reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            depth=10
        )
        
        # Publishers
        self.joint_state_pub = self.create_publisher(
            JointState,
            f'/{namespace}/joint_states',
            10
        )
        
        self.cartesian_pose_pub = self.create_publisher(
            PoseStamped,
            f'/{namespace}/cartesian_pose',
            10
        )
        
        # Subscribers
        self.joint_cmd_sub = self.create_subscription(
            JointState,
            f'/{namespace}/joint_command',
            self.joint_command_callback,
            reliable_qos
        )
        
        self.cartesian_cmd_sub = self.create_subscription(
            PoseStamped,
            f'/{namespace}/cartesian_command',
            self.cartesian_command_callback,
            reliable_qos
        )
        
        # Timer for publishing state
        self.create_timer(0.02, self.publish_state)  # 50 Hz
        
        # Timer for simulating motion
        self.create_timer(0.01, self.simulate_motion)  # 100 Hz
        
        self.get_logger().info(f'Mock robot "{namespace}" started with {num_joints} joints')
        self.get_logger().info(f'Publishing to:')
        self.get_logger().info(f'  - /{namespace}/joint_states')
        self.get_logger().info(f'  - /{namespace}/cartesian_pose')
        self.get_logger().info(f'Subscribing to:')
        self.get_logger().info(f'  - /{namespace}/joint_command')
        self.get_logger().info(f'  - /{namespace}/cartesian_command')
    
    def joint_command_callback(self, msg: JointState):
        """Handle joint command"""
        if len(msg.position) > 0:
            for i, pos in enumerate(msg.position):
                if i < self.num_joints:
                    self.target_positions[i] = pos
            self.get_logger().info(f'Received joint command: {list(msg.position)}')
    
    def cartesian_command_callback(self, msg: PoseStamped):
        """Handle Cartesian command"""
        self.position = {
            'x': msg.pose.position.x,
            'y': msg.pose.position.y,
            'z': msg.pose.position.z
        }
        self.orientation = {
            'w': msg.pose.orientation.w,
            'x': msg.pose.orientation.x,
            'y': msg.pose.orientation.y,
            'z': msg.pose.orientation.z
        }
        self.get_logger().info(
            f'Received Cartesian command: pos=({self.position["x"]:.3f}, '
            f'{self.position["y"]:.3f}, {self.position["z"]:.3f})'
        )
    
    def simulate_motion(self):
        """Simulate robot motion towards target"""
        # Simple P-controller simulation
        gain = 0.1
        for i in range(self.num_joints):
            error = self.target_positions[i] - self.joint_positions[i]
            velocity = gain * error
            self.joint_velocities[i] = velocity
            self.joint_positions[i] += velocity
            
            # Simulate effort
            self.joint_efforts[i] = error * 10.0
        
        # Update Cartesian pose based on joint positions (simplified FK)
        self._update_cartesian_from_joints()
    
    def _update_cartesian_from_joints(self):
        """Simple forward kinematics simulation"""
        # This is a very simplified FK just for demonstration
        # Real FK would depend on robot geometry
        base_x = 0.5
        base_y = 0.0
        base_z = 0.5
        
        # Use joint 1 and 2 to affect X and Y
        self.position['x'] = base_x + 0.3 * math.sin(self.joint_positions[0])
        self.position['y'] = base_y + 0.3 * math.sin(self.joint_positions[1])
        self.position['z'] = base_z + 0.2 * math.cos(self.joint_positions[2])
        
        # Simple rotation from joint 4, 5, 6
        angle = self.joint_positions[3]
        self.orientation['w'] = math.cos(angle / 2)
        self.orientation['x'] = 0.0
        self.orientation['y'] = 0.0
        self.orientation['z'] = math.sin(angle / 2)
    
    def publish_state(self):
        """Publish current state"""
        now = self.get_clock().now().to_msg()
        
        # Joint states
        joint_msg = JointState()
        joint_msg.header = Header()
        joint_msg.header.stamp = now
        joint_msg.header.frame_id = self.namespace
        joint_msg.name = self.joint_names
        joint_msg.position = self.joint_positions
        joint_msg.velocity = self.joint_velocities
        joint_msg.effort = self.joint_efforts
        
        self.joint_state_pub.publish(joint_msg)
        
        # Cartesian pose
        pose_msg = PoseStamped()
        pose_msg.header = Header()
        pose_msg.header.stamp = now
        pose_msg.header.frame_id = 'world'
        pose_msg.pose.position.x = self.position['x']
        pose_msg.pose.position.y = self.position['y']
        pose_msg.pose.position.z = self.position['z']
        pose_msg.pose.orientation.w = self.orientation['w']
        pose_msg.pose.orientation.x = self.orientation['x']
        pose_msg.pose.orientation.y = self.orientation['y']
        pose_msg.pose.orientation.z = self.orientation['z']
        
        self.cartesian_pose_pub.publish(pose_msg)


def main():
    parser = argparse.ArgumentParser(description='Mock ROS2 Robot for HMI Testing')
    parser.add_argument('--namespace', '-n', type=str, default='robot1',
                       help='Robot namespace (default: robot1)')
    parser.add_argument('--joints', '-j', type=int, default=6,
                       help='Number of joints (default: 6)')
    args = parser.parse_args()
    
    rclpy.init()
    
    node = MockRobotNode(args.namespace, args.joints)
    
    print(f"""
    ╔══════════════════════════════════════════════════════════╗
    ║         Mock Robot Node Started                          ║
    ║                                                          ║
    ║  Namespace: {args.namespace:<40} ║
    ║  Joints: {args.joints:<43} ║
    ║                                                          ║
    ║  Press Ctrl+C to stop                                    ║
    ╚══════════════════════════════════════════════════════════╝
    """)
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print('\nShutting down mock robot...')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
