"""
Flask Application for Isaac Sim HMI Dashboard
Provides REST API and WebSocket endpoints for robot control
With session-based authentication for security
"""

import os
import json
import atexit
import functools
from typing import Dict, Any, List
from flask import Flask, render_template, request, jsonify, redirect, url_for, flash
from flask_socketio import SocketIO, emit, disconnect
from flask_cors import CORS
from flask_login import LoginManager, UserMixin, login_user, logout_user, login_required, current_user
from werkzeug.security import generate_password_hash, check_password_hash

# ROS2 interface
from ros_interface import RobotManager
from ros_interface.robot_manager import get_robot_manager, shutdown_robot_manager


# Configuration
class Config:
    # Security: Generate a random secret key if not provided
    SECRET_KEY = os.environ.get('SECRET_KEY')
    if not SECRET_KEY:
        import secrets
        SECRET_KEY = secrets.token_hex(32)
        print("WARNING: Using auto-generated SECRET_KEY. Set SECRET_KEY env var for production.")
    
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
    
    # Security settings
    # Allowed CORS origins (comma-separated, or 'localhost' for local only)
    # Use '*' to allow all origins for local network HMI, or specify specific IPs
    CORS_ORIGINS = os.environ.get('CORS_ORIGINS', '*').split(',')
    # If using '*', note that CORS will allow all origins for local development
    # For production on local networks, you may want to restrict to specific IPs
    
    # Authentication credentials (set via environment variables!)
    # Default admin password is 'changeme' - CHANGE THIS IN PRODUCTION
    ADMIN_USERNAME = os.environ.get('HMI_ADMIN_USER', 'admin')
    ADMIN_PASSWORD_HASH = os.environ.get(
        'HMI_ADMIN_PASSWORD_HASH',
        generate_password_hash('changeme')  # Default password - CHANGE THIS!
    )
    
    # Session settings
    SESSION_COOKIE_SECURE = os.environ.get('SESSION_COOKIE_SECURE', 'False').lower() == 'true'
    SESSION_COOKIE_HTTPONLY = True
    SESSION_COOKIE_SAMESITE = 'Lax'


# ============== Robot Configuration ==============
# Configuration for specific robots with custom joint counts and names
ROBOT_CONFIGS = {
    'Plug_Bot': {
        'num_joints': 12,
        'joint_names': [
            # Arm joints (0-5)
            'joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6',
            # Gripper joints (6-11) - from openClose.yaml
            'finger_joint',
            'right_outer_knuckle_joint',
            'left_inner_finger_joint',
            'right_inner_finger_joint',
            'left_inner_finger_knuckle_joint',
            'right_inner_finger_knuckle_joint'
        ]
    }
}


# Initialize Flask app
app = Flask(__name__)
app.config.from_object(Config)


def get_cors_origins():
    """Get list of allowed CORS origins"""
    origins = Config.CORS_ORIGINS
    if '*' in origins:
        # Allow all origins (local network HMI)
        return '*'
    if 'localhost' in origins:
        # Expand localhost to common local development URLs
        return [
            f'http://localhost:{Config.PORT}',
            f'http://127.0.0.1:{Config.PORT}',
            f'https://localhost:{Config.PORT}',
            f'https://127.0.0.1:{Config.PORT}',
        ]
    return origins


# Configure CORS with restricted origins
CORS(app, origins=get_cors_origins(), supports_credentials=True)

# Initialize SocketIO with restricted origins
socketio = SocketIO(
    app, 
    cors_allowed_origins=get_cors_origins(),
    async_mode=Config.SOCKETIO_ASYNC_MODE
)

# Initialize Flask-Login
login_manager = LoginManager()
login_manager.init_app(app)
login_manager.login_view = 'login'
login_manager.login_message = 'Please log in to access the HMI dashboard.'
login_manager.login_message_category = 'warning'


# User model for authentication
class User(UserMixin):
    """Simple user model for HMI authentication"""
    def __init__(self, user_id, username):
        self.id = user_id
        self.username = username


# In-memory user store (single admin user)
users = {
    'admin': User('1', Config.ADMIN_USERNAME)
}


@login_manager.user_loader
def load_user(user_id):
    """Load user by ID for Flask-Login"""
    for user in users.values():
        if user.id == user_id:
            return user
    return None


def authenticated_only(f):
    """Decorator to require authentication for socket events"""
    @functools.wraps(f)
    def wrapped(*args, **kwargs):
        if not current_user.is_authenticated:
            disconnect()
            return None
        return f(*args, **kwargs)
    return wrapped


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


# ============== Authentication Routes ==============

@app.route('/login', methods=['GET', 'POST'])
def login():
    """Login page and handler"""
    if current_user.is_authenticated:
        return redirect(url_for('index'))
    
    if request.method == 'POST':
        username = request.form.get('username', '')
        password = request.form.get('password', '')
        
        # Validate credentials
        if username == Config.ADMIN_USERNAME and check_password_hash(Config.ADMIN_PASSWORD_HASH, password):
            user = users.get('admin')
            login_user(user, remember=True)
            
            # Redirect to requested page or dashboard
            next_page = request.args.get('next')
            if next_page and next_page.startswith('/'):
                return redirect(next_page)
            return redirect(url_for('index'))
        else:
            flash('Invalid username or password', 'error')
    
    return render_template('login.html')


@app.route('/logout')
@login_required
def logout():
    """Logout handler"""
    logout_user()
    flash('You have been logged out.', 'info')
    return redirect(url_for('login'))


# ============== Web Routes ==============

@app.route('/')
@login_required
def index():
    """Main dashboard page"""
    return render_template('index.html')


@app.route('/health')
def health():
    """Health check endpoint (public for monitoring)"""
    return jsonify({'status': 'ok', 'ros2_connected': robot_manager is not None})


# ============== REST API Routes ==============

@app.route('/api/robots', methods=['GET'])
@login_required
def get_robots():
    """Get list of connected robots"""
    manager = get_manager()
    return jsonify({
        'robots': manager.get_robot_list(),
        'success': True
    })


@app.route('/api/robots/scan', methods=['POST'])
@login_required
def scan_robots():
    """Scan for available robots"""
    manager = get_manager()
    try:
        discovered = manager.scan_for_robots()
        app.logger.info(f'Robot scan completed. Found: {discovered}')
        return jsonify({
            'discovered': discovered,
            'success': True
        })
    except Exception as e:
        app.logger.error(f'Robot scan error: {e}')
        return jsonify({
            'discovered': [],
            'success': False,
            'error': str(e)
        }), 500


@app.route('/api/robots/add', methods=['POST'])
@login_required
def add_robot():
    """Add a new robot to manage"""
    try:
        data = request.get_json()
        if not data:
            return jsonify({'success': False, 'error': 'No JSON data provided'}), 400
        
        namespace = data.get('namespace')
        num_joints = data.get('num_joints', 6)
        joint_names = data.get('joint_names')
        
        if not namespace:
            return jsonify({'success': False, 'error': 'Namespace required'}), 400
        
        app.logger.info(f'Adding robot: {namespace} with {num_joints} joints')
        
        # Check if this robot has a predefined configuration
        if namespace in ROBOT_CONFIGS:
            config = ROBOT_CONFIGS[namespace]
            num_joints = config.get('num_joints', 6)
            joint_names = config.get('joint_names')
            app.logger.info(f'Using predefined config for {namespace}')
        
        manager = get_manager()
        success = manager.add_robot(namespace, num_joints, joint_names)
        
        if success:
            app.logger.info(f'Successfully added robot: {namespace}')
            return jsonify({
                'success': True,
                'namespace': namespace,
                'num_joints': num_joints
            })
        else:
            # Robot likely already exists - check if it's in the manager
            if namespace in manager.get_robot_list():
                app.logger.info(f'Robot {namespace} already exists in manager, returning success')
                return jsonify({
                    'success': True,
                    'namespace': namespace,
                    'num_joints': num_joints,
                    'message': 'Robot already exists'
                })
            else:
                app.logger.warning(f'Failed to add robot {namespace}')
                return jsonify({
                    'success': False,
                    'error': 'Failed to add robot (ROS2 error)',
                    'namespace': namespace
                })
            
    except Exception as e:
        app.logger.error(f'Error adding robot: {e}', exc_info=True)
        return jsonify({
            'success': False,
            'error': str(e)
        }), 500


@app.route('/api/robots/remove', methods=['POST'])
@login_required
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
@login_required
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
@login_required
def get_all_robot_states():
    """Get states of all robots"""
    manager = get_manager()
    states = manager.get_all_states()
    
    return jsonify({
        'success': True,
        'states': states
    })


@app.route('/api/robot/<namespace>/joint_command', methods=['POST'])
@login_required
def send_joint_command(namespace: str):
    """Send joint command to a robot"""
    data = request.get_json()
    positions = data.get('positions', [])
    velocity = data.get('velocity')  # Optional velocity parameter
    
    if not positions:
        return jsonify({'success': False, 'error': 'Positions required'}), 400
    
    manager = get_manager()
    success = manager.send_joint_command(namespace, positions, velocity=velocity)
    
    return jsonify({
        'success': success,
        'namespace': namespace,
        'positions': positions,
        'velocity': velocity
    })





@app.route('/api/robot/<namespace>/joint_offset', methods=['POST'])
@login_required
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


@app.route('/api/robot/<namespace>/cartesian_command', methods=['POST'])
@login_required
def send_cartesian_command(namespace: str):
    """Send cartesian pose command to a robot"""
    data = request.get_json()
    position = data.get('position', {})
    orientation = data.get('orientation', {})
    
    if not position or not orientation:
        return jsonify({'success': False, 'error': 'Position and orientation required'}), 400
    
    manager = get_manager()
    success = manager.send_cartesian_command(namespace, position, orientation)
    
    return jsonify({
        'success': success,
        'namespace': namespace,
        'position': position,
        'orientation': orientation
    })


@app.route('/api/robot/<namespace>/cartesian_offset', methods=['POST'])
@login_required
def send_cartesian_offset(namespace: str):
    """Send cartesian offset (delta position) to a robot"""
    data = request.get_json()
    position_offset = data.get('position_offset', {})
    
    if not position_offset:
        return jsonify({'success': False, 'error': 'Position offset required'}), 400
    
    manager = get_manager()
    # Get current cartesian pose
    state = manager.get_robot_state(namespace)
    if not state or not state.get('cartesian_pose'):
        return jsonify({'success': False, 'error': 'Cannot get current cartesian pose'}), 400
    
    current_pose = state['cartesian_pose']
    # Apply offset to current position
    new_position = {
        'x': current_pose['position']['x'] + position_offset.get('x', 0),
        'y': current_pose['position']['y'] + position_offset.get('y', 0),
        'z': current_pose['position']['z'] + position_offset.get('z', 0),
    }
    
    # Keep orientation the same
    orientation = current_pose['orientation']
    
    success = manager.send_cartesian_command(namespace, new_position, orientation)
    
    return jsonify({
        'success': success,
        'namespace': namespace,
        'position_offset': position_offset,
        'new_position': new_position
    })





@app.route('/api/trigger/<int:group>', methods=['POST'])
@login_required
def send_trigger(group: int):
    """Send trigger pulse to a Cheese group"""
    if group < 1 or group > 4:
        return jsonify({'success': False, 'error': 'Group must be 1-4'}), 400
    
    manager = get_manager()
    success = manager.send_trigger(group)
    
    return jsonify({
        'success': success,
        'group': group
    })


@app.route('/api/trigger/roll-holder', methods=['POST'])
@login_required
def send_roll_holder_trigger():
    """Send trigger pulse to open Roll Holder"""
    manager = get_manager()
    success = manager.send_roll_holder_trigger()
    
    return jsonify({
        'success': success,
        'action': 'open_roll_holder'
    })


# ============== WebSocket Events ==============

@socketio.on('connect')
def handle_connect():
    """Handle WebSocket connection - requires authentication"""
    if not current_user.is_authenticated:
        print(f'Unauthorized WebSocket connection attempt: {request.sid}')
        disconnect()
        return False
    
    print(f'Client connected: {request.sid} (user: {current_user.username})')
    
    # Send current robot list
    manager = get_manager()
    emit('robot_list', {'robots': manager.get_robot_list()})


@socketio.on('disconnect')
def handle_disconnect():
    """Handle WebSocket disconnection"""
    print(f'Client disconnected: {request.sid}')


@socketio.on('subscribe_robot')
@authenticated_only
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
@authenticated_only
def handle_joint_command(data):
    """Handle joint command via WebSocket"""
    namespace = data.get('namespace')
    positions = data.get('positions', [])
    velocity = data.get('velocity')  # Optional velocity parameter
    
    manager = get_manager()
    success = manager.send_joint_command(namespace, positions, velocity=velocity)
    
    emit('command_result', {
        'type': 'joint_command',
        'success': success,
        'namespace': namespace
    })





@socketio.on('send_joint_offset')
@authenticated_only
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




@socketio.on('send_cartesian_command')
@authenticated_only
def handle_cartesian_command(data):
    """Handle cartesian command via WebSocket"""
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


@socketio.on('send_cartesian_offset')
@authenticated_only
def handle_cartesian_offset(data):
    """Handle cartesian offset via WebSocket"""
    namespace = data.get('namespace')
    position_offset = data.get('position_offset', {})
    
    manager = get_manager()
    
    # Get current cartesian pose
    state = manager.get_robot_state(namespace)
    if not state or not state.get('cartesian_pose'):
        emit('command_result', {
            'type': 'cartesian_offset',
            'success': False,
            'error': 'Cannot get current cartesian pose',
            'namespace': namespace
        })
        return
    
    current_pose = state['cartesian_pose']
    # Apply offset to current position
    new_position = {
        'x': current_pose['position']['x'] + position_offset.get('x', 0),
        'y': current_pose['position']['y'] + position_offset.get('y', 0),
        'z': current_pose['position']['z'] + position_offset.get('z', 0),
    }
    
    # Keep orientation the same
    orientation = current_pose['orientation']
    
    success = manager.send_cartesian_command(namespace, new_position, orientation)
    
    emit('command_result', {
        'type': 'cartesian_offset',
        'success': success,
        'namespace': namespace
    })



@socketio.on('send_trigger')
@authenticated_only
def handle_trigger(data):
    """Handle trigger command via WebSocket"""
    group = data.get('group')
    
    if not isinstance(group, int) or group < 1 or group > 4:
        emit('command_result', {
            'type': 'trigger',
            'success': False,
            'error': 'Invalid group'
        })
        return
    
    manager = get_manager()
    success = manager.send_trigger(group)
    
    emit('command_result', {
        'type': 'trigger',
        'success': success,
        'group': group
    })


@socketio.on('send_roll_holder_trigger')
@authenticated_only
def handle_roll_holder_trigger():
    """Handle roll holder trigger command via WebSocket"""
    manager = get_manager()
    success = manager.send_roll_holder_trigger()
    
    emit('command_result', {
        'type': 'roll_holder_trigger',
        'success': success,
        'action': 'open_roll_holder'
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
    ║         Isaac Sim HMI Dashboard (Secured)                ║
    ║                                                          ║
    ║  Starting server on http://{Config.HOST}:{Config.PORT}              ║
    ║                                                          ║
    ║  Predefined robots: {', '.join(Config.PREDEFINED_ROBOTS):<30} ║
    ║                                                          ║
    ║  SECURITY ENABLED:                                       ║
    ║  - Login required for dashboard access                   ║
    ║  - WebSocket authentication enforced                     ║
    ║  - CORS restricted to allowed origins                    ║
    ║                                                          ║
    ║  Default login: admin / changeme                         ║
    ║  (Set HMI_ADMIN_PASSWORD_HASH env var to change)         ║
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
