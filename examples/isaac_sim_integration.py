"""
Example Isaac Sim Script for HMI Integration
This script shows how to set up ROS2 topics in Isaac Sim that work with the HMI dashboard.

Run this in Isaac Sim's Script Editor or as a standalone application.
"""

import numpy as np
from typing import Optional

# Isaac Sim imports
import omni
from omni.isaac.core import World
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.core.articulations import Articulation
from omni.isaac.core.utils.nucleus import get_assets_root_path

# ROS2 imports
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header


class IsaacSimRobotBridge(Node):
    """
    ROS2 Node that bridges Isaac Sim robot with ROS2 topics.
    Publishes joint_states and cartesian_pose.
    Subscribes to joint_command and cartesian_command.
    """
    
    def __init__(self, robot_name: str, articulation: Articulation):
        super().__init__(f'{robot_name}_bridge')
        
        self.robot_name = robot_name
        self.articulation = articulation
        self.num_joints = articulation.num_dof
        
        # Get joint names from articulation (matches Isaac Sim convention)
        # These will be overridden by actual joint names from the articulation if available
        self.joint_names = [f'joint{i+1}' for i in range(self.num_joints)]
        
        # Try to get actual joint names from articulation
        try:
            dof_names = articulation.dof_names
            if dof_names:
                self.joint_names = list(dof_names)
        except Exception:
            pass  # Use default names
        
        # Publishers
        self.joint_state_pub = self.create_publisher(
            JointState,
            f'/{robot_name}/joint_states',
            10
        )
        
        self.cartesian_pose_pub = self.create_publisher(
            PoseStamped,
            f'/{robot_name}/cartesian_pose',
            10
        )
        
        # Subscribers
        self.joint_cmd_sub = self.create_subscription(
            JointState,
            f'/{robot_name}/joint_command',
            self.joint_command_callback,
            10
        )
        
        self.cartesian_cmd_sub = self.create_subscription(
            PoseStamped,
            f'/{robot_name}/cartesian_command',
            self.cartesian_command_callback,
            10
        )
        
        # Store latest commands
        self.target_positions: Optional[np.ndarray] = None
        self.target_pose: Optional[PoseStamped] = None
        
        self.get_logger().info(f'Isaac Sim Robot Bridge initialized for {robot_name}')
    
    def joint_command_callback(self, msg: JointState):
        """Handle incoming joint commands"""
        if len(msg.position) > 0:
            self.target_positions = np.array(msg.position[:self.num_joints])
            self.get_logger().debug(f'Received joint command: {self.target_positions}')
    
    def cartesian_command_callback(self, msg: PoseStamped):
        """Handle incoming Cartesian commands"""
        self.target_pose = msg
        self.get_logger().debug(
            f'Received Cartesian command: pos=({msg.pose.position.x}, '
            f'{msg.pose.position.y}, {msg.pose.position.z})'
        )
        # Note: Implementing Cartesian control requires IK solver
        # This is robot-specific and beyond this example
    
    def publish_state(self):
        """Publish current robot state"""
        # Get current joint positions
        joint_positions = self.articulation.get_joint_positions()
        joint_velocities = self.articulation.get_joint_velocities()
        
        # Publish joint states
        joint_msg = JointState()
        joint_msg.header = Header()
        joint_msg.header.stamp = self.get_clock().now().to_msg()
        joint_msg.header.frame_id = self.robot_name
        joint_msg.name = self.joint_names
        joint_msg.position = joint_positions.tolist() if joint_positions is not None else []
        joint_msg.velocity = joint_velocities.tolist() if joint_velocities is not None else []
        
        self.joint_state_pub.publish(joint_msg)
        
        # Publish Cartesian pose (end-effector)
        # This requires knowing the end-effector link
        # For this example, we'll compute a simple FK or use a fixed transform
        pose_msg = PoseStamped()
        pose_msg.header = Header()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = 'world'
        
        # Get end-effector pose (simplified - actual implementation depends on robot)
        # You would typically use the robot's FK or get the pose of the end-effector prim
        ee_pose = self._get_end_effector_pose()
        if ee_pose is not None:
            pose_msg.pose.position.x = ee_pose[0]
            pose_msg.pose.position.y = ee_pose[1]
            pose_msg.pose.position.z = ee_pose[2]
            pose_msg.pose.orientation.w = ee_pose[3]
            pose_msg.pose.orientation.x = ee_pose[4]
            pose_msg.pose.orientation.y = ee_pose[5]
            pose_msg.pose.orientation.z = ee_pose[6]
        
        self.cartesian_pose_pub.publish(pose_msg)
    
    def _get_end_effector_pose(self) -> Optional[np.ndarray]:
        """
        Get end-effector pose. Override this for your specific robot.
        Returns: [x, y, z, qw, qx, qy, qz]
        """
        try:
            # Get the world pose of the end-effector
            # This assumes the articulation has a method to get body poses
            # Actual implementation depends on your robot setup
            
            # Example using body index (typically the last link)
            # body_poses = self.articulation.get_body_poses()
            # ee_pose = body_poses[-1]  # Last link
            
            # Placeholder - return identity pose
            return np.array([0.5, 0.0, 0.5, 1.0, 0.0, 0.0, 0.0])
        except Exception as e:
            self.get_logger().warn(f'Could not get end-effector pose: {e}')
            return None
    
    def apply_commands(self):
        """Apply stored commands to the robot"""
        if self.target_positions is not None:
            self.articulation.set_joint_position_targets(self.target_positions)


def setup_robot_bridge(world: World, robot_prim_path: str, robot_name: str) -> IsaacSimRobotBridge:
    """
    Set up a robot bridge for the given robot in Isaac Sim.
    
    Args:
        world: Isaac Sim World instance
        robot_prim_path: USD path to the robot (e.g., "/World/robot1")
        robot_name: Name/namespace for ROS2 topics
        
    Returns:
        IsaacSimRobotBridge instance
    """
    # Get the articulation
    articulation = world.scene.get_object(robot_name)
    
    if articulation is None:
        # Try to add it
        articulation = world.scene.add(
            Articulation(
                prim_path=robot_prim_path,
                name=robot_name
            )
        )
    
    # Initialize ROS2 if needed
    if not rclpy.ok():
        rclpy.init()
    
    # Create the bridge
    bridge = IsaacSimRobotBridge(robot_name, articulation)
    
    return bridge


# ============== Example Usage ==============

def main():
    """
    Main function demonstrating HMI integration with Isaac Sim.
    Run this in Isaac Sim's script editor.
    """
    # Initialize the world
    world = World(stage_units_in_meters=1.0)
    
    # Get assets root path
    assets_root = get_assets_root_path()
    
    # Add a robot (example: Franka Panda)
    robot_usd = assets_root + "/Isaac/Robots/Franka/franka_alt_fingers.usd"
    add_reference_to_stage(robot_usd, "/World/robot1")
    
    # Add robot to scene
    robot = world.scene.add(
        Articulation(
            prim_path="/World/robot1",
            name="robot1"
        )
    )
    
    # Reset world to initialize physics
    world.reset()
    
    # Set up the ROS2 bridge
    bridge = setup_robot_bridge(world, "/World/robot1", "robot1")
    
    # Simulation loop
    print("Starting simulation with ROS2 bridge...")
    print("Access the HMI at http://localhost:5000")
    print("Press Ctrl+C to stop")
    
    try:
        while True:
            # Step simulation
            world.step(render=True)
            
            # Spin ROS2 (non-blocking)
            rclpy.spin_once(bridge, timeout_sec=0)
            
            # Publish robot state
            bridge.publish_state()
            
            # Apply any received commands
            bridge.apply_commands()
            
    except KeyboardInterrupt:
        print("Shutting down...")
    finally:
        bridge.destroy_node()
        rclpy.shutdown()
        world.stop()


if __name__ == "__main__":
    main()
