#!/bin/bash
# Installation script for Jetson Nano (Ubuntu 18.04) with ROS 2 Foxy
# This script installs all dependencies needed for the wbk_yolo package

set -e  # Exit on error

echo "=========================================="
echo "Jetson Nano ROS 2 Foxy Installation"
echo "=========================================="
echo ""

# Check Ubuntu version
UBUNTU_VERSION=$(lsb_release -rs)
echo "Detected Ubuntu version: $UBUNTU_VERSION"

if [[ "$UBUNTU_VERSION" != "18.04" ]]; then
    echo "WARNING: This script is designed for Ubuntu 18.04"
    echo "Your system: Ubuntu $UBUNTU_VERSION"
    read -p "Continue anyway? (y/n) " -n 1 -r
    echo
    if [[ ! $REPLY =~ ^[Yy]$ ]]; then
        exit 1
    fi
fi

# 1. Install ROS 2 Foxy (if not already installed)
echo ""
echo "Step 1: Checking ROS 2 Foxy installation..."
if dpkg -l | grep -q ros-foxy-desktop; then
    echo "✓ ROS 2 Foxy is already installed"
else
    echo "Installing ROS 2 Foxy..."
    
    # Add ROS 2 repository
    sudo apt update && sudo apt install -y curl gnupg2 lsb-release
    sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
    
    if [ ! -f /etc/apt/sources.list.d/ros2-latest.list ]; then
        sudo sh -c 'echo "deb http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2-latest.list'
    fi
    
    sudo apt update
    sudo apt install -y ros-foxy-desktop
    
    echo "✓ ROS 2 Foxy installed"
fi

# 2. Install ROS 2 dependencies
echo ""
echo "Step 2: Installing ROS 2 package dependencies..."
sudo apt install -y \
    ros-foxy-rclpy \
    ros-foxy-sensor-msgs \
    ros-foxy-std-msgs \
    ros-foxy-geometry-msgs \
    ros-foxy-cv-bridge \
    ros-foxy-image-transport \
    python3-opencv \
    python3-numpy \
    python3-yaml \
    python3-pip

echo "✓ ROS 2 dependencies installed"

# 3. Install Python dependencies
echo ""
echo "Step 3: Installing Python dependencies..."

# Install PyTorch for Jetson (check JetPack version)
echo "Installing PyTorch for Jetson..."
# Note: Adjust version based on your JetPack version
# For JetPack 4.x (Ubuntu 18.04), use:
pip3 install torch torchvision --index-url https://download.pytorch.org/whl/cu111

# For JetPack 5.x, use:
# pip3 install --pre torch torchvision torchaudio --index-url https://download.pytorch.org/whl/cu121

echo "Installing ultralytics..."
pip3 install ultralytics

echo "✓ Python dependencies installed"

# 4. Setup ROS 2 environment
echo ""
echo "Step 4: Setting up ROS 2 environment..."
if ! grep -q "source /opt/ros/foxy/setup.bash" ~/.bashrc; then
    echo "" >> ~/.bashrc
    echo "# ROS 2 Foxy" >> ~/.bashrc
    echo "source /opt/ros/foxy/setup.bash" >> ~/.bashrc
    echo "✓ Added ROS 2 Foxy to ~/.bashrc"
else
    echo "✓ ROS 2 Foxy already in ~/.bashrc"
fi

# 5. Verify installation
echo ""
echo "Step 5: Verifying installation..."
source /opt/ros/foxy/setup.bash

echo "Checking ROS 2 version..."
ros2 --version || echo "WARNING: ros2 command not found"

echo "Checking Python packages..."
python3 -c "import rclpy; print('✓ rclpy:', rclpy.__version__)" || echo "✗ rclpy not found"
python3 -c "import cv2; print('✓ OpenCV:', cv2.__version__)" || echo "✗ OpenCV not found"
python3 -c "import numpy; print('✓ NumPy:', numpy.__version__)" || echo "✗ NumPy not found"
python3 -c "import torch; print('✓ PyTorch:', torch.__version__)" || echo "✗ PyTorch not found"
python3 -c "from ultralytics import YOLO; print('✓ Ultralytics installed')" || echo "✗ Ultralytics not found"

echo ""
echo "=========================================="
echo "Installation complete!"
echo "=========================================="
echo ""
echo "Next steps:"
echo "1. Source your workspace: source install/setup.bash"
echo "2. Build your package: colcon build --packages-select wbk_yolo"
echo "3. Run: ros2 launch wbk_yolo vision.launch.py"
echo ""
echo "Note: If you get import errors, make sure to:"
echo "  - Source ROS 2: source /opt/ros/foxy/setup.bash"
echo "  - Source your workspace: source install/setup.bash"
echo ""


