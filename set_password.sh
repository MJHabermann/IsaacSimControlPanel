#!/bin/bash
# Helper script to generate and set HMI password
# Usage: ./set_password.sh [password]
#        If no password provided, will prompt interactively

set -e

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
VENV_DIR="$SCRIPT_DIR/.venv"

# Colors
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
CYAN='\033[0;36m'
NC='\033[0m'

echo -e "${GREEN}╔══════════════════════════════════════════════════════════╗${NC}"
echo -e "${GREEN}║         HMI Password Configuration Tool                  ║${NC}"
echo -e "${GREEN}╚══════════════════════════════════════════════════════════╝${NC}"
echo ""

# Activate venv if it exists (for werkzeug)
if [ -d "$VENV_DIR" ]; then
    source "$VENV_DIR/bin/activate"
fi

# Check for werkzeug
if ! python3 -c "import werkzeug" 2>/dev/null; then
    echo "Installing werkzeug..."
    pip install werkzeug >/dev/null 2>&1
fi

# Get password
if [ -n "$1" ]; then
    PASSWORD="$1"
else
    echo -e "${CYAN}Enter new password:${NC}"
    read -s PASSWORD
    echo ""
    echo -e "${CYAN}Confirm password:${NC}"
    read -s PASSWORD_CONFIRM
    echo ""
    
    if [ "$PASSWORD" != "$PASSWORD_CONFIRM" ]; then
        echo -e "\033[0;31mPasswords do not match!${NC}"
        exit 1
    fi
fi

if [ -z "$PASSWORD" ]; then
    echo -e "\033[0;31mPassword cannot be empty!${NC}"
    exit 1
fi

# Generate hash
HASH=$(python3 -c "from werkzeug.security import generate_password_hash; print(generate_password_hash('$PASSWORD'))")

echo ""
echo -e "${GREEN}Password hash generated successfully!${NC}"
echo ""
echo -e "${YELLOW}Option 1: Set environment variable before running:${NC}"
echo ""
echo "  export HMI_ADMIN_PASSWORD_HASH='$HASH'"
echo "  ./launch.sh"
echo ""
echo -e "${YELLOW}Option 2: Add to your .env file or launch.sh:${NC}"
echo ""
echo "  HMI_ADMIN_PASSWORD_HASH='$HASH'"
echo ""

# Ask if user wants to update launch.sh
echo -e "${CYAN}Would you like to add this to launch.sh? (y/N):${NC}"
read -r REPLY

if [[ "$REPLY" =~ ^[Yy]$ ]]; then
    # Check if already has a password hash line
    if grep -q "HMI_ADMIN_PASSWORD_HASH" "$SCRIPT_DIR/launch.sh"; then
        # Replace existing line
        sed -i "s|export HMI_ADMIN_PASSWORD_HASH=.*|export HMI_ADMIN_PASSWORD_HASH='$HASH'|" "$SCRIPT_DIR/launch.sh"
        echo -e "${GREEN}Updated existing password in launch.sh${NC}"
    else
        # Add before the python3 app.py line
        sed -i "/^python3 app.py/i export HMI_ADMIN_PASSWORD_HASH='$HASH'" "$SCRIPT_DIR/launch.sh"
        echo -e "${GREEN}Added password to launch.sh${NC}"
    fi
    echo ""
    echo -e "${GREEN}Done! Run ./launch.sh to start with the new password.${NC}"
fi
