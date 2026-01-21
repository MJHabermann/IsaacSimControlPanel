"""
ROS2 Subscribers for Robot State Feedback
Handles subscribing to joint_states and cartesian_pose topics
"""

import threading
from typing import Dict, Any, Optional, Callable, List
from dataclasses import dataclass, field
from datetime import datetime

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose


@dataclass
class RobotState:
    """Thread-safe container for robot state data"""
    joint_names: list = field(default_factory=list)
    joint_positions: list = field(default_factory=list)
    joint_velocities: list = field(default_factory=list)
    joint_efforts: list = field(default_factory=list)
    
    cartesian_position: Dict[str, float] = field(default_factory=lambda: {'x': 0.0, 'y': 0.0, 'z': 0.0})
    cartesian_orientation: Dict[str, float] = field(default_factory=lambda: {'w': 1.0, 'x': 0.0, 'y': 0.0, 'z': 0.0})
    
    joint_timestamp: Optional[datetime] = None
    cartesian_timestamp: Optional[datetime] = None
    
    def to_dict(self) -> Dict[str, Any]:
        """Convert state to dictionary for JSON serialization"""
        return {
            'joint_states': {
                'names': self.joint_names,
                'positions': self.joint_positions,
                'velocities': self.joint_velocities,
                'efforts': self.joint_efforts,
                'timestamp': self.joint_timestamp.isoformat() if self.joint_timestamp else None
            },
            'cartesian_pose': {
                'position': self.cartesian_position,
                'orientation': self.cartesian_orientation,
                'timestamp': self.cartesian_timestamp.isoformat() if self.cartesian_timestamp else None
            }
        }


class RobotSubscriber:
    """
    Manages ROS2 subscriptions for a single robot namespace.
    Thread-safe access to latest robot state.
    """
    
    def __init__(self, node: Node, robot_namespace: str):
        """
        Initialize subscriber for a robot.
        
        Args:
            node: ROS2 node instance
            robot_namespace: Robot namespace (e.g., 'robot1')
        """
        self.node = node
        self.namespace = robot_namespace
        self._state = RobotState()
        self._lock = threading.Lock()
        self._callbacks: list[Callable] = []
        
        # QoS profile for sensor data
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Create subscriptions
        joint_topic = f'/{robot_namespace}/joint_states'
        pose_topic = f'/{robot_namespace}/cartesian_pose'
        
        self._joint_sub = node.create_subscription(
            JointState,
            joint_topic,
            self._joint_states_callback,
            qos_profile
        )
        
        self._pose_sub = node.create_subscription(
            Pose,
            pose_topic,
            self._cartesian_pose_callback,
            qos_profile
        )
        
        node.get_logger().info(f'Subscribed to {joint_topic} and {pose_topic}')
    
    def _joint_states_callback(self, msg: JointState):
        """Handle incoming joint state messages"""
        with self._lock:
            self._state.joint_names = list(msg.name)
            self._state.joint_positions = list(msg.position)
            self._state.joint_velocities = list(msg.velocity) if msg.velocity else []
            self._state.joint_efforts = list(msg.effort) if msg.effort else []
            self._state.joint_timestamp = datetime.now()
        
        # Notify callbacks
        for callback in self._callbacks:
            try:
                callback('joint_states', self.get_state())
            except Exception as e:
                self.node.get_logger().error(f'Callback error: {e}')
    
    def _cartesian_pose_callback(self, msg: Pose):
        """Handle incoming Cartesian pose messages"""
        with self._lock:
            self._state.cartesian_position = {
                'x': msg.position.x,
                'y': msg.position.y,
                'z': msg.position.z
            }
            self._state.cartesian_orientation = {
                'w': msg.orientation.w,
                'x': msg.orientation.x,
                'y': msg.orientation.y,
                'z': msg.orientation.z
            }
            self._state.cartesian_timestamp = datetime.now()
        
        # Notify callbacks
        for callback in self._callbacks:
            try:
                callback('cartesian_pose', self.get_state())
            except Exception as e:
                self.node.get_logger().error(f'Callback error: {e}')
    
    def get_state(self) -> Dict[str, Any]:
        """Get current robot state (thread-safe)"""
        with self._lock:
            return self._state.to_dict()
    
    def get_joint_names(self) -> List[str]:
        """Get the joint names received from the robot (thread-safe)"""
        with self._lock:
            return list(self._state.joint_names)
    
    def has_joint_data(self) -> bool:
        """Check if we have received joint data"""
        with self._lock:
            return len(self._state.joint_names) > 0
    
    def add_callback(self, callback: Callable):
        """Add a callback for state updates"""
        self._callbacks.append(callback)
    
    def remove_callback(self, callback: Callable):
        """Remove a callback"""
        if callback in self._callbacks:
            self._callbacks.remove(callback)
    
    def destroy(self):
        """Clean up subscriptions"""
        self.node.destroy_subscription(self._joint_sub)
        self.node.destroy_subscription(self._pose_sub)
        self._callbacks.clear()
