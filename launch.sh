#!/bin/bash
# Launch script for Isaac Sim HMI Dashboard
# This script ensures ROS2 is sourced and starts the Flask server

set -e

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

echo -e "${GREEN}"
echo "╔══════════════════════════════════════════════════════════╗"
echo "║         Isaac Sim HMI Dashboard Launcher                 ║"
echo "╚══════════════════════════════════════════════════════════╝"
echo -e "${NC}"

# Check for ROS2 installation
if [ -z "$ROS_DISTRO" ]; then
    echo -e "${YELLOW}ROS2 not sourced. Attempting to source...${NC}"
    
    # Try common ROS2 locations
    if [ -f "/opt/ros/jazzy/setup.bash" ]; then
        source /opt/ros/jazzy/setup.bash
        echo -e "${GREEN}Sourced ROS2 Jazzy${NC}"
    elif [ -f "/opt/ros/iron/setup.bash" ]; then
        source /opt/ros/iron/setup.bash
        echo -e "${GREEN}Sourced ROS2 Iron${NC}"
    elif [ -f "/opt/ros/rolling/setup.bash" ]; then
        source /opt/ros/rolling/setup.bash
        echo -e "${GREEN}Sourced ROS2 Rolling${NC}"
    else
        echo -e "${RED}ERROR: Could not find ROS2 installation.${NC}"
        echo "Please source your ROS2 setup.bash manually before running this script."
        exit 1
    fi
else
    echo -e "${GREEN}Using ROS2 $ROS_DISTRO${NC}"
fi

# Get script directory
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

# Check for Python
if ! command -v python3 &> /dev/null; then
    echo -e "${RED}ERROR: Python3 not found${NC}"
    exit 1
fi

# Locate Python — find first venv whose Python can actually import Flask
VENV_DIR="$SCRIPT_DIR/.venv"
FALLBACK_VENV="/home/isaacsim/Desktop/IsaacSimControlPanel/.venv"
PYTHON=""

for candidate in "$VENV_DIR/bin/python3" "$FALLBACK_VENV/bin/python3"; do
    if [ -x "$candidate" ] && "$candidate" -c "import flask, flask_login" 2>/dev/null; then
        PYTHON="$candidate"
        echo -e "${GREEN}Using Python: $PYTHON${NC}"
        break
    fi
done

if [ -z "$PYTHON" ]; then
    echo -e "${RED}ERROR: Could not find a Python with Flask installed.${NC}"
    echo "Checked: $VENV_DIR/bin/python3"
    echo "         $FALLBACK_VENV/bin/python3"
    exit 1
fi

# Check for rclpy (provided by ROS2, not the venv)
if ! "$PYTHON" -c "import rclpy" 2>/dev/null; then
    echo -e "${RED}ERROR: rclpy not found. Make sure ROS2 Python packages are installed.${NC}"
    exit 1
fi

# Default configuration
export FLASK_HOST="${FLASK_HOST:-0.0.0.0}"
export FLASK_PORT="${FLASK_PORT:-5000}"
export PREDEFINED_ROBOTS="${PREDEFINED_ROBOTS:-}"

echo ""
echo "Configuration:"
echo "  Host: $FLASK_HOST"
echo "  Port: $FLASK_PORT"
echo "  Predefined Robots: $PREDEFINED_ROBOTS"
echo ""
echo -e "${GREEN}Starting HMI Dashboard...${NC}"
echo -e "Access at: ${YELLOW}http://localhost:$FLASK_PORT${NC}"
echo ""

# Run the Flask app
cd "$SCRIPT_DIR"
export HMI_ADMIN_PASSWORD_HASH='scrypt:32768:8:1$yOsIi8xzpcxajNL4$32a6fe37acd1aee36212bd6083d22703f4d54bbac1b0526eba7fc8414cbc2a27914087f6edf772f5b564c4cab4ae20128c4e2345e1b67e770edefe4e346f5049'
"$PYTHON" app.py
