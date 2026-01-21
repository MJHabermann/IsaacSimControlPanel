"""
ROS2 Publishers for Robot Commands
Handles publishing joint_command and cartesian_command topics
"""

import threading
from typing import Dict, List, Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose
from std_msgs.msg import Header


class RobotPublisher:
    """
    Manages ROS2 publishers for a single robot namespace.
    Thread-safe publishing of commands.
    """
    
    def __init__(self, node: Node, robot_namespace: str, num_joints: int = 6):
        """
        Initialize publishers for a robot.
        
        Args:
            node: ROS2 node instance
            robot_namespace: Robot namespace (e.g., 'robot1')
            num_joints: Number of joints (default 6 for 6-axis robot)
        """
        self.node = node
        self.namespace = robot_namespace
        self.num_joints = num_joints
        self._lock = threading.Lock()
        
        # Default joint names for a 6-axis robot (matches Isaac Sim convention)
        self._joint_names = [f'joint{i+1}' for i in range(num_joints)]
        self._joint_names_initialized = False
        
        # QoS profile for commands - MUST be RELIABLE to match Isaac Sim's subscriber
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Create publishers
        joint_topic = f'/{robot_namespace}/joint_command'
        cartesian_topic = f'/{robot_namespace}/cartesian_command'
        
        self._joint_pub = node.create_publisher(
            JointState,
            joint_topic,
            qos_profile
        )
        
        self._cartesian_pub = node.create_publisher(
            Pose,
            cartesian_topic,
            qos_profile
        )
        
        node.get_logger().info(f'Created publishers for {joint_topic} and {cartesian_topic}')
    
    def set_joint_names(self, names: List[str]):
        """Set custom joint names (usually from subscriber feedback)"""
        with self._lock:
            self._joint_names = list(names)
            self.num_joints = len(names)
            self._joint_names_initialized = True
            self.node.get_logger().info(f'Joint names set to: {names}')
    
    def get_joint_names(self) -> List[str]:
        """Get current joint names"""
        with self._lock:
            return list(self._joint_names)
    
    def is_joint_names_initialized(self) -> bool:
        """Check if joint names have been initialized from feedback"""
        with self._lock:
            return self._joint_names_initialized
    
    def publish_joint_command(self, positions: List[float], 
                               velocities: Optional[List[float]] = None,
                               efforts: Optional[List[float]] = None) -> bool:
        """
        Publish joint command.
        
        Args:
            positions: List of joint positions (radians). Can be partial (e.g., 6 for arm only)
            velocities: Optional list of joint velocities
            efforts: Optional list of joint efforts/torques
            
        Returns:
            True if published successfully
        """
        # Allow partial joint commands (e.g., 6 joints for arm, ignoring gripper)
        # Only check that we don't exceed the number of joints
        if len(positions) > self.num_joints:
            self.node.get_logger().error(
                f'Too many positions: got {len(positions)}, max {self.num_joints}'
            )
            return False
        
        with self._lock:
            msg = JointState()
            msg.header = Header()
            msg.header.stamp = self.node.get_clock().now().to_msg()
            msg.header.frame_id = self.namespace
            
            # Only include joint names for the positions we're sending
            msg.name = self._joint_names[:len(positions)]
            msg.position = [float(p) for p in positions]
            msg.velocity = [float(v) for v in velocities] if velocities else []
            msg.effort = [float(e) for e in efforts] if efforts else []
            
            try:
                self._joint_pub.publish(msg)
                self.node.get_logger().debug(f'Published joint command: {positions}')
                return True
            except Exception as e:
                self.node.get_logger().error(f'Failed to publish joint command: {e}')
                return False
    
    def publish_cartesian_command(self, 
                                   position: Dict[str, float],
                                   orientation: Dict[str, float],
                                   frame_id: str = 'world') -> bool:
        """
        Publish Cartesian pose command.
        
        Args:
            position: Dict with 'x', 'y', 'z' keys (meters)
            orientation: Dict with 'w', 'x', 'y', 'z' keys (quaternion)
            frame_id: Reference frame
            
        Returns:
            True if published successfully
        """
        with self._lock:
            msg = Pose()
            
            msg.position.x = float(position.get('x', 0.0))
            msg.position.y = float(position.get('y', 0.0))
            msg.position.z = float(position.get('z', 0.0))
            
            msg.orientation.w = float(orientation.get('w', 1.0))
            msg.orientation.x = float(orientation.get('x', 0.0))
            msg.orientation.y = float(orientation.get('y', 0.0))
            msg.orientation.z = float(orientation.get('z', 0.0))
            
            try:
                self._cartesian_pub.publish(msg)
                self.node.get_logger().debug(
                    f'Published cartesian command: pos={position}, orient={orientation}'
                )
                return True
            except Exception as e:
                self.node.get_logger().error(f'Failed to publish cartesian command: {e}')
                return False
    
    def publish_joint_offset(self, current_positions: List[float], 
                              offsets: List[float]) -> bool:
        """
        Apply offset to current joint positions and publish.
        
        Args:
            current_positions: Current joint positions
            offsets: Offsets to apply (can be positive or negative)
            
        Returns:
            True if published successfully
        """
        # Allow partial offsets (e.g., 6 for arm only)
        num_offsets = len(offsets)
        if num_offsets > len(current_positions):
            self.node.get_logger().error(
                f'Too many offsets: got {num_offsets}, current positions has {len(current_positions)}'
            )
            return False
        
        new_positions = [
            curr + off for curr, off in zip(current_positions[:num_offsets], offsets)
        ]
        return self.publish_joint_command(new_positions)
    
    def publish_cartesian_offset(self,
                                  current_position: Dict[str, float],
                                  current_orientation: Dict[str, float],
                                  position_offset: Optional[Dict[str, float]] = None,
                                  orientation_offset: Optional[Dict[str, float]] = None) -> bool:
        """
        Apply offset to current Cartesian pose and publish.
        
        Args:
            current_position: Current position dict
            current_orientation: Current orientation dict
            position_offset: Position offset to apply
            orientation_offset: Orientation offset (simple addition, not proper quaternion math)
            
        Returns:
            True if published successfully
        """
        new_position = current_position.copy()
        new_orientation = current_orientation.copy()
        
        if position_offset:
            for key in ['x', 'y', 'z']:
                new_position[key] = current_position.get(key, 0.0) + position_offset.get(key, 0.0)
        
        if orientation_offset:
            # Note: For proper quaternion operations, use a quaternion library
            # This is simplified for small adjustments
            for key in ['w', 'x', 'y', 'z']:
                new_orientation[key] = current_orientation.get(key, 0.0) + orientation_offset.get(key, 0.0)
            
            # Normalize quaternion
            norm = sum(v**2 for v in new_orientation.values()) ** 0.5
            if norm > 0:
                for key in new_orientation:
                    new_orientation[key] /= norm
        
        return self.publish_cartesian_command(new_position, new_orientation)
    
    def destroy(self):
        """Clean up publishers"""
        self.node.destroy_publisher(self._joint_pub)
        self.node.destroy_publisher(self._cartesian_pub)
