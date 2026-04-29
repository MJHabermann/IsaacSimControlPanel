# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

Flask-based web HMI (Human-Machine Interface) for controlling robots in NVIDIA Isaac Sim via ROS2. Provides a browser UI with joint-slider and Cartesian controls, real-time WebSocket state feedback, and domain-specific controls (stopper droppers, conveyors, roll holders, cutter door).

## Running the Application

```bash
# Full startup (Linux/Unix with ROS2)
source /opt/ros/humble/setup.bash
./launch.sh

# Manual startup
source /opt/ros/humble/setup.bash
source .venv/bin/activate
python app.py
```

The venv must be created with `--system-site-packages` so that `rclpy` (installed by ROS2) is accessible:
```bash
python3 -m venv .venv --system-site-packages
pip install -r requirements.txt
```

### Testing Without ROS2

```bash
python examples/test_mock_robot.py
```

## Key Environment Variables

| Variable | Default | Notes |
|---|---|---|
| `PREDEFINED_ROBOTS` | `robot1,robot2` | Comma-separated ROS2 namespace list |
| `HMI_ADMIN_PASSWORD_HASH` | Default hash | Werkzeug scrypt hash; default password is `changeme` |
| `FLASK_HOST` / `FLASK_PORT` | `0.0.0.0` / `5000` | Bind address |
| `FLASK_DEBUG` | `False` | Never enable in production |

## Architecture

### Backend (`app.py` + `ros_interface/`)

```
app.py  ──────────────────────────────────────────────────
  Config, Flask-Login auth, 40+ REST routes, SocketIO events
       │
       ▼
ros_interface/robot_manager.py  (RobotManager singleton)
  ├── robot_manager.py  — ROS2 node, MultiThreadedExecutor in daemon thread,
  │                        robot lifecycle, state callbacks → SocketIO emissions
  ├── publishers.py     — RobotPublisher: joint & Cartesian command publishing
  └── subscribers.py    — RobotSubscriber: joint states & Cartesian pose; RobotState dataclass
```

**Thread model:** Flask/SocketIO runs on the main thread; ROS2 spin runs in a background daemon thread using `MultiThreadedExecutor`. All shared state is protected with `threading.RLock()`. Never call `rclpy.spin()` from Flask handlers — use `manager.send_*()` methods which are thread-safe.

**Trigger pulse pattern:** Boolean triggers (stoppers, doors, roll holder) publish `True` then `False` with a 100 ms delay — this is intentional, not a bug.

### Frontend (`templates/` + `static/`)

- `templates/index.html` — Main dashboard (dev vs. expo mode toggled via session)
- `static/scripts.js` — All client logic: Socket.IO connection, slider handlers, command dispatch, state update rendering
- `static/styles.css` — All styling

Display mode (`dev` or `expo`) is set at `/display-type` post-login and stored in the Flask session. Dev mode shows joint sliders; expo mode hides them.

### ROS2 Topic Conventions

| Direction | Topic pattern | Message type |
|---|---|---|
| Subscribe | `/{ns}/joint_states` | `sensor_msgs/JointState` |
| Publish | `/{ns}/joint_command` | `sensor_msgs/JointState` |
| Subscribe | `/{ns}/cartesian_pose` | `Float64MultiArray` |
| Publish | `/{ns}/cartesian_command` | `Float64MultiArray` |
| Publish | `/Cheese/group{1-5}` | `std_msgs/Bool` (trigger pulse) |
| Subscribe | `/beam_1/state`, `/beam_2/state` | `std_msgs/Bool` (sensors) |

### Robot Configuration

Hard-coded joint configs live in `ROBOT_CONFIGS` dict in `app.py`. `Plug_Bot` is the primary config (12 joints). Adding a new robot type requires updating this dict and the corresponding frontend slider generation in `index.html`.

### Saved Positions

`POST /api/save-position` writes JSON snapshots to `instance/positions/`. These files accumulate — no automatic cleanup.

## Authentication Flow

1. All routes except `/login` and `/api/health` require Flask-Login session.
2. WebSocket connections are gated by `@authenticated_only` decorator — unauthenticated sockets are immediately disconnected.
3. Default credentials: `admin` / `changeme` — must be replaced via `HMI_ADMIN_PASSWORD_HASH` env var before any shared deployment.

## Common Gotchas

- **`rclpy` not found:** venv was not created with `--system-site-packages`, or ROS2 wasn't sourced before activating the venv.
- **SocketIO async mode:** Uses `threading` mode (not `eventlet` or `gevent`). Do not switch async modes without testing; the background ROS2 executor depends on standard threading.
- **Robot namespace mismatch:** The namespace in `PREDEFINED_ROBOTS` must exactly match what Isaac Sim publishes (case-sensitive).
