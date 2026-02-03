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

# Check for Flask
if ! python3 -c "import flask" 2>/dev/null; then
    echo -e "${YELLOW}Flask not installed. Installing requirements...${NC}"
    pip install -r "$SCRIPT_DIR/requirements.txt"
fi

# Check for rclpy
if ! python3 -c "import rclpy" 2>/dev/null; then
    echo -e "${RED}ERROR: rclpy not found. Make sure ROS2 Python packages are installed.${NC}"
    exit 1
fi

# Default configuration
export FLASK_HOST="${FLASK_HOST:-0.0.0.0}"
export FLASK_PORT="${FLASK_PORT:-5000}"
export PREDEFINED_ROBOTS="${PREDEFINED_ROBOTS:-robot1,robot2}"

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
python3 app.py
