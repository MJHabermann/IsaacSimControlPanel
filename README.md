# Isaac Sim HMI Dashboard

A Flask-based web interface for controlling robots in NVIDIA Isaac Sim via ROS2. This HMI (Human-Machine Interface) provides a browser-based control dashboard for monitoring and commanding multiple robots.

## Features

- 🤖 **Multi-Robot Support**: Manage multiple robots simultaneously with namespace switching
- 🎛️ **Joint Control**: 6-axis slider controls with real-time feedback
- 📍 **Cartesian Control**: Position and orientation (quaternion) inputs
- 📊 **Live Feedback**: Auto-refreshing joint states and Cartesian pose display
- 🔌 **WebSocket Streaming**: Real-time updates via Socket.IO
- 🧵 **Thread-Safe**: Non-blocking ROS2 integration with proper thread safety

## Project Structure

```
isaacSimHMI/
├── app.py                    # Flask application entry point
├── requirements.txt          # Python dependencies
├── README.md                 # This file
├── static/
│   ├── scripts.js           # JavaScript for UI controls
│   └── styles.css           # CSS styling
├── templates/
│   └── index.html           # Main dashboard HTML
└── ros_interface/
    ├── __init__.py          # Package init
    ├── robot_manager.py     # Central robot management
    ├── publishers.py        # ROS2 publishers
    └── subscribers.py       # ROS2 subscribers
```

## Prerequisites

1. **ROS2 Installation** (Humble or later recommended)
   ```bash
   # Source ROS2
   source /opt/ros/humble/setup.bash
   ```

2. **Isaac Sim** with ROS2 Bridge enabled
   - Isaac Sim 2023.1.0 or later
   - ROS2 Bridge extension enabled

3. **Python 3.8+** with pip

## Installation

1. Clone or copy the project:
   ```bash
   cd ~/isaacSimHMI
   ```

2. Install Python dependencies:
   ```bash
   pip install -r requirements.txt
   ```

3. Source your ROS2 workspace:
   ```bash
   source /opt/ros/humble/setup.bash
   # If you have a custom workspace:
   source ~/your_ws/install/setup.bash
   ```

## Running the Application

### Step 1: Start Isaac Sim with ROS2 Bridge

1. Launch Isaac Sim
2. Load your robot scene
3. Enable the ROS2 Bridge extension
4. Ensure your robot is publishing/subscribing to the expected topics:
   - `/{robot_name}/joint_states` (sensor_msgs/JointState)
   - `/{robot_name}/cartesian_pose` (geometry_msgs/PoseStamped)
   - `/{robot_name}/joint_command` (sensor_msgs/JointState)
   - `/{robot_name}/cartesian_command` (geometry_msgs/PoseStamped)

### Step 2: Start the HMI Dashboard

```bash
# Source ROS2 first!
source /opt/ros/humble/setup.bash

# Set environment variables (optional)
export PREDEFINED_ROBOTS="robot1,robot2"  # Comma-separated robot namespaces
export FLASK_PORT=5000
export FLASK_HOST=0.0.0.0

# Run the application
python app.py
```

### Step 3: Access the Dashboard

Open a web browser and navigate to:
```
http://localhost:5000
```

## Configuration

### Environment Variables

| Variable | Default | Description |
|----------|---------|-------------|
| `FLASK_HOST` | `0.0.0.0` | Server bind address |
| `FLASK_PORT` | `5000` | Server port |
| `FLASK_DEBUG` | `False` | Enable debug mode |
| `PREDEFINED_ROBOTS` | `robot1,robot2` | Comma-separated robot namespaces |
| `SECRET_KEY` | `isaac-sim-hmi-secret-key` | Flask secret key |

### Adding Robots

1. **Via Web UI**: Enter the robot namespace in the input field and click "Add Robot"
2. **Via Environment Variable**: Set `PREDEFINED_ROBOTS` before starting
3. **Via Scan**: Click "Scan" to discover robots publishing to ROS2 topics

## API Reference

### REST Endpoints

| Method | Endpoint | Description |
|--------|----------|-------------|
| GET | `/api/robots` | List connected robots |
| POST | `/api/robots/scan` | Scan for available robots |
| POST | `/api/robots/add` | Add a robot by namespace |
| POST | `/api/robots/remove` | Remove a robot |
| GET | `/api/robot/<ns>/state` | Get robot state |
| POST | `/api/robot/<ns>/joint_command` | Send joint command |
| POST | `/api/robot/<ns>/cartesian_command` | Send Cartesian command |
| POST | `/api/robot/<ns>/joint_offset` | Apply joint offset |
| POST | `/api/robot/<ns>/cartesian_offset` | Apply Cartesian offset |

### WebSocket Events

| Event | Direction | Description |
|-------|-----------|-------------|
| `robot_list` | Server → Client | List of connected robots |
| `robot_state` | Server → Client | Robot state update |
| `command_result` | Server → Client | Command execution result |
| `subscribe_robot` | Client → Server | Subscribe to robot updates |
| `send_joint_command` | Client → Server | Send joint command |
| `send_cartesian_command` | Client → Server | Send Cartesian command |

## Isaac Sim Integration

### Example Isaac Sim Python Script

Here's an example of setting up ROS2 topics in Isaac Sim:

```python
import omni
from omni.isaac.core import World
from omni.isaac.core.robots import Robot
from omni.isaac.ros2_bridge import ROS2Bridge

# Initialize world
world = World()

# Load your robot
robot = world.scene.add(Robot(
    prim_path="/World/robot1",
    name="robot1"
))

# The ROS2 Bridge extension will automatically create topics:
# /robot1/joint_states - publishes current joint positions
# /robot1/joint_command - subscribes to joint commands

# For Cartesian pose, you may need to add a custom publisher
# using the ArticulationView or compute forward kinematics
```

### Topic Naming Convention

The HMI expects topics in this format:
- `/{namespace}/joint_states` - JointState messages with current positions
- `/{namespace}/joint_command` - JointState messages for commanding
- `/{namespace}/cartesian_pose` - PoseStamped with end-effector pose
- `/{namespace}/cartesian_command` - PoseStamped for Cartesian control

## UI Improvements (Optional)

### Suggested Enhancements

1. **Tabbed Interface**: Add tabs for each robot instead of a dropdown
   ```javascript
   // Example implementation in scripts.js
   function createRobotTabs() {
       // Create tab elements for each robot
   }
   ```

2. **3D Visualization**: Integrate Three.js for robot visualization
   ```html
   <script src="https://cdnjs.cloudflare.com/ajax/libs/three.js/r128/three.min.js"></script>
   ```

3. **Joystick Control**: Add virtual joystick for Cartesian jog
   ```javascript
   // Use nipplejs library for virtual joystick
   ```

4. **Save/Load Poses**: Store and recall robot positions
   ```javascript
   // Use localStorage or server-side storage
   ```

5. **Safety Limits**: Add configurable joint limits and warnings
   ```javascript
   // Implement limit checking before sending commands
   ```

## Troubleshooting

### Common Issues

1. **"No module named 'rclpy'"**
   - Make sure to source ROS2 before running: `source /opt/ros/humble/setup.bash`

2. **"Connection refused" in browser**
   - Check if the server is running on the correct port
   - Verify firewall settings

3. **"No robots found" during scan**
   - Verify Isaac Sim ROS2 Bridge is running
   - Check topic names with `ros2 topic list`

4. **Laggy updates**
   - Reduce update rate in the UI
   - Check network latency
   - Ensure ROS2 domain ID matches

### Debug Mode

Enable debug logging:
```bash
export FLASK_DEBUG=True
python app.py
```

## License

MIT License - Feel free to modify and distribute.

## Contributing

Contributions are welcome! Please submit issues and pull requests.
# IsaacSimControlPanel
