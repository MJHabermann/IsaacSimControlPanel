"""
Flask Application for Isaac Sim HMI Dashboard
Provides REST API and WebSocket endpoints for robot control
"""

import os
import json
import atexit
from typing import Dict, Any, List
from flask import Flask, render_template, request, jsonify
from flask_socketio import SocketIO, emit
from flask_cors import CORS

# ROS2 interface
from ros_interface import RobotManager
from ros_interface.robot_manager import get_robot_manager, shutdown_robot_manager

# Configuration
class Config:
    SECRET_KEY = os.environ.get('SECRET_KEY', 'isaac-sim-hmi-secret-key')
    DEBUG = os.environ.get('FLASK_DEBUG', 'False').lower() == 'true'
    HOST = os.environ.get('FLASK_HOST', '0.0.0.0')
    PORT = int(os.environ.get('FLASK_PORT', 5000))
    
    # Predefined robot namespaces (can be overridden by environment variable)
    PREDEFINED_ROBOTS = os.environ.get(
        'PREDEFINED_ROBOTS', 
        'robot1,robot2'
    ).split(',')
    
    # WebSocket settings
    SOCKETIO_ASYNC_MODE = 'threading'


# Initialize Flask app
app = Flask(__name__)
app.config.from_object(Config)
CORS(app)

# Initialize SocketIO with threading mode for ROS2 compatibility
socketio = SocketIO(
    app, 
    cors_allowed_origins="*",
    async_mode=Config.SOCKETIO_ASYNC_MODE
)

# Robot manager (initialized lazily)
robot_manager: RobotManager = None


def get_manager() -> RobotManager:
    """Get or initialize the robot manager"""
    global robot_manager
    if robot_manager is None:
        robot_manager = get_robot_manager(Config.PREDEFINED_ROBOTS)
        
        # Add WebSocket callback for state streaming
        robot_manager.add_state_callback(on_robot_state_update)
    
    return robot_manager


def on_robot_state_update(namespace: str, topic: str, state: Dict):
    """Callback for robot state updates - broadcasts via WebSocket"""
    socketio.emit('robot_state', {
        'namespace': namespace,
        'topic': topic,
        'state': state
    })


# ============== Web Routes ==============

@app.route('/')
def index():
    """Main dashboard page"""
    return render_template('index.html')


@app.route('/health')
def health():
    """Health check endpoint"""
    return jsonify({'status': 'ok', 'ros2_connected': robot_manager is not None})


# ============== REST API Routes ==============

@app.route('/api/robots', methods=['GET'])
def get_robots():
    """Get list of connected robots"""
    manager = get_manager()
    return jsonify({
        'robots': manager.get_robot_list(),
        'success': True
    })


@app.route('/api/robots/scan', methods=['POST'])
def scan_robots():
    """Scan for available robots"""
    manager = get_manager()
    discovered = manager.scan_for_robots()
    return jsonify({
        'discovered': discovered,
        'success': True
    })


@app.route('/api/robots/add', methods=['POST'])
def add_robot():
    """Add a new robot to manage"""
    data = request.get_json()
    namespace = data.get('namespace')
    num_joints = data.get('num_joints', 6)
    joint_names = data.get('joint_names')
    
    if not namespace:
        return jsonify({'success': False, 'error': 'Namespace required'}), 400
    
    manager = get_manager()
    success = manager.add_robot(namespace, num_joints, joint_names)
    
    return jsonify({
        'success': success,
        'namespace': namespace
    })


@app.route('/api/robots/remove', methods=['POST'])
def remove_robot():
    """Remove a robot from management"""
    data = request.get_json()
    namespace = data.get('namespace')
    
    if not namespace:
        return jsonify({'success': False, 'error': 'Namespace required'}), 400
    
    manager = get_manager()
    success = manager.remove_robot(namespace)
    
    return jsonify({'success': success})


@app.route('/api/robot/<namespace>/state', methods=['GET'])
def get_robot_state(namespace: str):
    """Get current state of a specific robot"""
    manager = get_manager()
    state = manager.get_robot_state(namespace)
    
    if state is None:
        return jsonify({'success': False, 'error': 'Robot not found'}), 404
    
    return jsonify({
        'success': True,
        'namespace': namespace,
        'state': state
    })


@app.route('/api/robots/states', methods=['GET'])
def get_all_robot_states():
    """Get states of all robots"""
    manager = get_manager()
    states = manager.get_all_states()
    
    return jsonify({
        'success': True,
        'states': states
    })


@app.route('/api/robot/<namespace>/joint_command', methods=['POST'])
def send_joint_command(namespace: str):
    """Send joint command to a robot"""
    data = request.get_json()
    positions = data.get('positions', [])
    
    if not positions:
        return jsonify({'success': False, 'error': 'Positions required'}), 400
    
    manager = get_manager()
    success = manager.send_joint_command(namespace, positions)
    
    return jsonify({
        'success': success,
        'namespace': namespace,
        'positions': positions
    })


@app.route('/api/robot/<namespace>/cartesian_command', methods=['POST'])
def send_cartesian_command(namespace: str):
    """Send Cartesian command to a robot"""
    data = request.get_json()
    position = data.get('position', {})
    orientation = data.get('orientation', {})
    
    if not position or not orientation:
        return jsonify({
            'success': False, 
            'error': 'Position and orientation required'
        }), 400
    
    manager = get_manager()
    success = manager.send_cartesian_command(namespace, position, orientation)
    
    return jsonify({
        'success': success,
        'namespace': namespace,
        'position': position,
        'orientation': orientation
    })


@app.route('/api/robot/<namespace>/joint_offset', methods=['POST'])
def send_joint_offset(namespace: str):
    """Apply joint offsets to current position"""
    data = request.get_json()
    offsets = data.get('offsets', [])
    
    if not offsets:
        return jsonify({'success': False, 'error': 'Offsets required'}), 400
    
    manager = get_manager()
    success = manager.send_joint_offset(namespace, offsets)
    
    return jsonify({
        'success': success,
        'namespace': namespace,
        'offsets': offsets
    })


@app.route('/api/robot/<namespace>/cartesian_offset', methods=['POST'])
def send_cartesian_offset(namespace: str):
    """Apply Cartesian offsets to current pose"""
    data = request.get_json()
    position_offset = data.get('position_offset')
    orientation_offset = data.get('orientation_offset')
    
    manager = get_manager()
    success = manager.send_cartesian_offset(
        namespace, position_offset, orientation_offset
    )
    
    return jsonify({
        'success': success,
        'namespace': namespace
    })


# ============== WebSocket Events ==============

@socketio.on('connect')
def handle_connect():
    """Handle WebSocket connection"""
    print(f'Client connected: {request.sid}')
    
    # Send current robot list
    manager = get_manager()
    emit('robot_list', {'robots': manager.get_robot_list()})


@socketio.on('disconnect')
def handle_disconnect():
    """Handle WebSocket disconnection"""
    print(f'Client disconnected: {request.sid}')


@socketio.on('subscribe_robot')
def handle_subscribe(data):
    """Subscribe to a robot's state updates"""
    namespace = data.get('namespace')
    print(f'Client {request.sid} subscribed to {namespace}')
    
    # Send current state immediately
    manager = get_manager()
    state = manager.get_robot_state(namespace)
    if state:
        emit('robot_state', {
            'namespace': namespace,
            'topic': 'initial',
            'state': state
        })


@socketio.on('send_joint_command')
def handle_joint_command(data):
    """Handle joint command via WebSocket"""
    namespace = data.get('namespace')
    positions = data.get('positions', [])
    
    manager = get_manager()
    success = manager.send_joint_command(namespace, positions)
    
    emit('command_result', {
        'type': 'joint_command',
        'success': success,
        'namespace': namespace
    })


@socketio.on('send_cartesian_command')
def handle_cartesian_command(data):
    """Handle Cartesian command via WebSocket"""
    namespace = data.get('namespace')
    position = data.get('position', {})
    orientation = data.get('orientation', {})
    
    manager = get_manager()
    success = manager.send_cartesian_command(namespace, position, orientation)
    
    emit('command_result', {
        'type': 'cartesian_command',
        'success': success,
        'namespace': namespace
    })


@socketio.on('send_joint_offset')
def handle_joint_offset(data):
    """Handle joint offset via WebSocket"""
    namespace = data.get('namespace')
    offsets = data.get('offsets', [])
    
    manager = get_manager()
    success = manager.send_joint_offset(namespace, offsets)
    
    emit('command_result', {
        'type': 'joint_offset',
        'success': success,
        'namespace': namespace
    })


@socketio.on('send_cartesian_offset')
def handle_cartesian_offset(data):
    """Handle Cartesian offset via WebSocket"""
    namespace = data.get('namespace')
    position_offset = data.get('position_offset')
    orientation_offset = data.get('orientation_offset')
    
    manager = get_manager()
    success = manager.send_cartesian_offset(
        namespace, position_offset, orientation_offset
    )
    
    emit('command_result', {
        'type': 'cartesian_offset',
        'success': success,
        'namespace': namespace
    })


# ============== Cleanup ==============

@atexit.register
def cleanup():
    """Cleanup on application exit"""
    shutdown_robot_manager()


# ============== Main Entry Point ==============

if __name__ == '__main__':
    print(f"""
    ╔══════════════════════════════════════════════════════════╗
    ║         Isaac Sim HMI Dashboard                          ║
    ║                                                          ║
    ║  Starting server on http://{Config.HOST}:{Config.PORT}              ║
    ║                                                          ║
    ║  Predefined robots: {', '.join(Config.PREDEFINED_ROBOTS):<30} ║
    ╚══════════════════════════════════════════════════════════╝
    """)
    
    # Initialize robot manager at startup
    get_manager()
    
    # Run with SocketIO
    socketio.run(
        app, 
        host=Config.HOST, 
        port=Config.PORT, 
        debug=Config.DEBUG,
        allow_unsafe_werkzeug=True  # For development
    )
