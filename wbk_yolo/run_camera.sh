#!/bin/bash
# Run script for USB Camera Node
# Run this in Terminal 1

set -e

# Colors
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m' # No Color

echo "=========================================="
echo "USB Camera Node Launcher"
echo "=========================================="
echo ""

# Source ROS2
if [ -f /opt/ros/eloquent/setup.bash ]; then
    source /opt/ros/eloquent/setup.bash
    echo -e "${GREEN}Sourced ROS2 Eloquent${NC}"
elif [ -f /opt/ros/foxy/setup.bash ]; then
    source /opt/ros/foxy/setup.bash
    echo -e "${GREEN}Sourced ROS2 Foxy${NC}"
else
    echo -e "${RED}ROS2 not found. Please install ROS2 first.${NC}"
    exit 1
fi

# Source workspace
if [ -f ~/ros2_ws/install/setup.bash ]; then
    source ~/ros2_ws/install/setup.bash
    echo -e "${GREEN}Sourced workspace${NC}"
else
    echo -e "${YELLOW}Workspace not built. Building now...${NC}"
    cd ~/ros2_ws
    colcon build --packages-select wbk_vision --symlink-install
    source install/setup.bash
    cd -
fi

# Check if camera node is already running
if pgrep -f "usb_camera_node" > /dev/null; then
    echo -e "${YELLOW}Camera node already running${NC}"
    read -p "Kill existing camera node? (y/n) " -n 1 -r
    echo
    if [[ $REPLY =~ ^[Yy]$ ]]; then
        pkill -f "usb_camera_node"
        sleep 1
    else
        echo "Exiting. Camera node already running."
        exit 0
    fi
fi

# Check camera device
if [ ! -e /dev/video0 ]; then
    echo -e "${RED}Camera device /dev/video0 not found${NC}"
    echo "Available video devices:"
    ls -l /dev/video* 2>/dev/null || echo "No video devices found"
    exit 1
fi

echo ""
echo -e "${GREEN}Starting USB camera node...${NC}"
echo "Publishing to: /camera/image_raw"
echo "Press Ctrl+C to stop"
echo ""

# Run camera node
ros2 run wbk_vision usb_camera_node

