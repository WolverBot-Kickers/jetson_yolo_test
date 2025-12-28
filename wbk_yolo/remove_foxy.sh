#!/bin/bash
# Script to remove ROS2 Foxy and clean up

set -e

# Colors
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m' # No Color

echo "=========================================="
echo "ROS2 Foxy Removal Script"
echo "=========================================="
echo ""
echo "This script will:"
echo "  1. Remove ROS2 Foxy packages"
echo "  2. Remove Foxy from ~/.bashrc"
echo "  3. Clean up Foxy installation directory"
echo ""
read -p "Continue? (y/n) " -n 1 -r
echo
if [[ ! $REPLY =~ ^[Yy]$ ]]; then
    exit 0
fi

echo ""
echo "Step 1: Removing ROS2 Foxy packages..."
sudo apt remove --purge -y \
    ros-foxy-* \
    python3-argcomplete || true

sudo apt autoremove -y

echo ""
echo "Step 2: Removing Foxy from ~/.bashrc..."
if grep -q "source /opt/ros/foxy/setup.bash" ~/.bashrc; then
    sed -i '/source \/opt\/ros\/foxy\/setup.bash/d' ~/.bashrc
    echo -e "${GREEN}Removed Foxy from ~/.bashrc${NC}"
else
    echo -e "${YELLOW}Foxy not found in ~/.bashrc${NC}"
fi

echo ""
echo "Step 3: Removing Foxy installation directory..."
if [ -d "/opt/ros/foxy" ]; then
    sudo rm -rf /opt/ros/foxy
    echo -e "${GREEN}Removed /opt/ros/foxy${NC}"
else
    echo -e "${YELLOW}/opt/ros/foxy not found${NC}"
fi

echo ""
echo "Step 4: Cleaning up ROS2 repository (if only Foxy was installed)..."
# Check if other ROS2 versions are installed
if ! dpkg -l | grep -q "ros-eloquent" && ! dpkg -l | grep -q "ros-dashing"; then
    echo "No other ROS2 versions found. Removing repository..."
    sudo rm -f /etc/apt/sources.list.d/ros2-latest.list
    sudo apt-key del 0xC1CF6E31E6BADE8868B172B4F42ED6FBAB17C654 2>/dev/null || true
    echo -e "${GREEN}Removed ROS2 repository${NC}"
else
    echo -e "${YELLOW}Other ROS2 versions detected. Keeping repository.${NC}"
fi

echo ""
echo "=========================================="
echo -e "${GREEN}ROS2 Foxy removal complete!${NC}"
echo "=========================================="
echo ""
echo "Next steps:"
echo "  1. Install ROS2 Eloquent: ./install_jetson_yolo.sh"
echo "  2. Log out and back in (or: source ~/.bashrc)"
echo ""

