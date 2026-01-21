"""
Robot Manager - Central ROS2 Interface for Multiple Robots
Manages publishers/subscribers and provides thread-safe access
"""

import threading
import time
from typing import Dict, List, Any, Optional, Callable
from dataclasses import dataclass
import queue

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor

from .publishers import RobotPublisher
from .subscribers import RobotSubscriber


@dataclass
class RobotConfig:
    """Configuration for a robot"""
    namespace: str
    num_joints: int = 6
    joint_names: Optional[List[str]] = None


class RobotManagerNode(Node):
    """ROS2 Node for managing multiple robots"""
    
    def __init__(self):
        super().__init__('isaac_sim_hmi_node')
        self.get_logger().info('Isaac Sim HMI Node initialized')


class RobotManager:
    """
    Manages multiple robot connections with thread-safe access.
    Runs ROS2 spinning in a background thread.
    """
    
    def __init__(self, predefined_robots: Optional[List[str]] = None):
        """
        Initialize the Robot Manager.
        
        Args:
            predefined_robots: List of robot namespaces to connect to initially
        """
        self._robots: Dict[str, Dict[str, Any]] = {}
        self._lock = threading.RLock()
        self._running = False
        self._spin_thread: Optional[threading.Thread] = None
        self._executor: Optional[MultiThreadedExecutor] = None
        self._node: Optional[RobotManagerNode] = None
        self._state_callbacks: List[Callable] = []
        self._command_queue = queue.Queue()
        
        # Initialize ROS2
        if not rclpy.ok():
            rclpy.init()
        
        # Create node and executor
        self._node = RobotManagerNode()
        self._executor = MultiThreadedExecutor(num_threads=4)
        self._executor.add_node(self._node)
        
        # Start spinning in background
        self._start_spinning()
        
        # Add predefined robots
        if predefined_robots:
            for namespace in predefined_robots:
                self.add_robot(namespace)
    
    def _start_spinning(self):
        """Start ROS2 executor in background thread"""
        if self._running:
            return
        
        self._running = True
        self._spin_thread = threading.Thread(target=self._spin_loop, daemon=True)
        self._spin_thread.start()
        self._node.get_logger().info('ROS2 spinning started in background')
    
    def _spin_loop(self):
        """Background thread for ROS2 spinning"""
        while self._running and rclpy.ok():
            try:
                self._executor.spin_once(timeout_sec=0.1)
            except Exception as e:
                if self._running:
                    self._node.get_logger().error(f'Spin error: {e}')
    
    def add_robot(self, namespace: str, num_joints: int = 6, 
                  joint_names: Optional[List[str]] = None) -> bool:
        """
        Add a robot to manage.
        
        Args:
            namespace: Robot namespace
            num_joints: Number of joints
            joint_names: Optional custom joint names
            
        Returns:
            True if added successfully
        """
        with self._lock:
            if namespace in self._robots:
                self._node.get_logger().warn(f'Robot {namespace} already exists')
                return False
            
            try:
                publisher = RobotPublisher(self._node, namespace, num_joints)
                subscriber = RobotSubscriber(self._node, namespace)
                
                if joint_names:
                    publisher.set_joint_names(joint_names)
                
                # Add callback to forward state updates and sync joint names
                def on_state_callback(topic, state, ns=namespace):
                    # Sync joint names from subscriber to publisher on first receive
                    self._sync_joint_names(ns)
                    self._on_state_update(ns, topic, state)
                
                subscriber.add_callback(on_state_callback)
                
                self._robots[namespace] = {
                    'publisher': publisher,
                    'subscriber': subscriber,
                    'config': RobotConfig(namespace, num_joints, joint_names)
                }
                
                self._node.get_logger().info(f'Added robot: {namespace}')
                return True
                
            except Exception as e:
                self._node.get_logger().error(f'Failed to add robot {namespace}: {e}')
                return False
    
    def remove_robot(self, namespace: str) -> bool:
        """Remove a robot from management"""
        with self._lock:
            if namespace not in self._robots:
                return False
            
            robot = self._robots[namespace]
            robot['subscriber'].destroy()
            robot['publisher'].destroy()
            del self._robots[namespace]
            
            self._node.get_logger().info(f'Removed robot: {namespace}')
            return True
    
    def get_robot_list(self) -> List[str]:
        """Get list of managed robot namespaces"""
        with self._lock:
            return list(self._robots.keys())
    
    def get_robot_state(self, namespace: str) -> Optional[Dict[str, Any]]:
        """
        Get current state of a robot.
        
        Args:
            namespace: Robot namespace
            
        Returns:
            Robot state dict or None if not found
        """
        with self._lock:
            if namespace not in self._robots:
                return None
            return self._robots[namespace]['subscriber'].get_state()
    
    def get_all_states(self) -> Dict[str, Dict[str, Any]]:
        """Get states of all robots"""
        with self._lock:
            return {
                ns: robot['subscriber'].get_state()
                for ns, robot in self._robots.items()
            }
    
    def send_joint_command(self, namespace: str, positions: List[float]) -> bool:
        """
        Send joint command to a robot.
        
        Args:
            namespace: Robot namespace
            positions: Joint positions in radians
            
        Returns:
            True if sent successfully
        """
        with self._lock:
            if namespace not in self._robots:
                return False
            return self._robots[namespace]['publisher'].publish_joint_command(positions)
    
    def send_cartesian_command(self, namespace: str, 
                                position: Dict[str, float],
                                orientation: Dict[str, float]) -> bool:
        """
        Send Cartesian command to a robot.
        
        Args:
            namespace: Robot namespace
            position: Position dict with x, y, z
            orientation: Orientation dict with w, x, y, z
            
        Returns:
            True if sent successfully
        """
        with self._lock:
            if namespace not in self._robots:
                return False
            return self._robots[namespace]['publisher'].publish_cartesian_command(
                position, orientation
            )
    
    def send_joint_offset(self, namespace: str, offsets: List[float]) -> bool:
        """
        Apply joint offsets to current position.
        
        Args:
            namespace: Robot namespace
            offsets: Offset values in radians
            
        Returns:
            True if sent successfully
        """
        with self._lock:
            if namespace not in self._robots:
                return False
            
            # Get current state
            state = self._robots[namespace]['subscriber'].get_state()
            current_positions = state['joint_states']['positions']
            
            if not current_positions:
                self._node.get_logger().warn(f'No current joint positions for {namespace}')
                return False
            
            return self._robots[namespace]['publisher'].publish_joint_offset(
                current_positions, offsets
            )
    
    def send_cartesian_offset(self, namespace: str,
                               position_offset: Optional[Dict[str, float]] = None,
                               orientation_offset: Optional[Dict[str, float]] = None) -> bool:
        """
        Apply Cartesian offsets to current pose.
        
        Args:
            namespace: Robot namespace
            position_offset: Position offset dict
            orientation_offset: Orientation offset dict
            
        Returns:
            True if sent successfully
        """
        with self._lock:
            if namespace not in self._robots:
                return False
            
            # Get current state
            state = self._robots[namespace]['subscriber'].get_state()
            current_pos = state['cartesian_pose']['position']
            current_orient = state['cartesian_pose']['orientation']
            
            return self._robots[namespace]['publisher'].publish_cartesian_offset(
                current_pos, current_orient, position_offset, orientation_offset
            )
    
    def add_state_callback(self, callback: Callable):
        """Add a callback for state updates (for WebSocket streaming)"""
        self._state_callbacks.append(callback)
    
    def remove_state_callback(self, callback: Callable):
        """Remove a state callback"""
        if callback in self._state_callbacks:
            self._state_callbacks.remove(callback)
    
    def _sync_joint_names(self, namespace: str):
        """
        Sync joint names from subscriber to publisher.
        Called when we receive joint_states to ensure publisher uses correct names.
        """
        if namespace not in self._robots:
            return
        
        robot = self._robots[namespace]
        publisher = robot['publisher']
        subscriber = robot['subscriber']
        
        # Only sync if publisher hasn't been initialized yet
        if not publisher.is_joint_names_initialized() and subscriber.has_joint_data():
            joint_names = subscriber.get_joint_names()
            if joint_names:
                publisher.set_joint_names(joint_names)
                self._node.get_logger().info(
                    f'Synced joint names for {namespace}: {joint_names}'
                )
    
    def _on_state_update(self, namespace: str, topic: str, state: Dict):
        """Internal callback for state updates"""
        for callback in self._state_callbacks:
            try:
                callback(namespace, topic, state)
            except Exception as e:
                self._node.get_logger().error(f'State callback error: {e}')
    
    def scan_for_robots(self, timeout: float = 2.0) -> List[str]:
        """
        Scan for available robot namespaces by looking for topics.
        
        Args:
            timeout: Time to wait for topic discovery
            
        Returns:
            List of discovered robot namespaces
        """
        discovered = set()
        
        # Get all topics
        topic_list = self._node.get_topic_names_and_types()
        
        for topic_name, _ in topic_list:
            # Look for joint_states topics to identify robots
            if topic_name.endswith('/joint_states'):
                # Extract namespace
                parts = topic_name.split('/')
                if len(parts) >= 2:
                    namespace = parts[1]
                    discovered.add(namespace)
        
        return list(discovered)
    
    def shutdown(self):
        """Shutdown the robot manager"""
        self._running = False
        
        with self._lock:
            # Clean up all robots
            for namespace in list(self._robots.keys()):
                self.remove_robot(namespace)
        
        # Wait for spin thread
        if self._spin_thread and self._spin_thread.is_alive():
            self._spin_thread.join(timeout=2.0)
        
        # Shutdown executor and node
        if self._executor:
            self._executor.shutdown()
        
        if self._node:
            self._node.destroy_node()
        
        # Don't call rclpy.shutdown() here as other nodes might be using it
        print('Robot Manager shutdown complete')


# Singleton instance for Flask app
_robot_manager: Optional[RobotManager] = None


def get_robot_manager(predefined_robots: Optional[List[str]] = None) -> RobotManager:
    """
    Get or create the singleton RobotManager instance.
    
    Args:
        predefined_robots: List of robot namespaces (only used on first call)
        
    Returns:
        RobotManager instance
    """
    global _robot_manager
    if _robot_manager is None:
        _robot_manager = RobotManager(predefined_robots)
    return _robot_manager


def shutdown_robot_manager():
    """Shutdown the singleton RobotManager"""
    global _robot_manager
    if _robot_manager:
        _robot_manager.shutdown()
        _robot_manager = None
