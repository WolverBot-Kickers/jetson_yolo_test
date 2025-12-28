#!/bin/bash
# Installation script for YOLO vision stack on Jetson Nano (Ubuntu 18.04 + ROS2 Foxy)
# This script installs all dependencies needed for wbk_yolo package

set -e  # Exit on error

echo "=========================================="
echo "YOLO Vision Stack Installation for Jetson"
echo "=========================================="
echo ""

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Check if running on Jetson
if [ ! -f /etc/nv_tegra_release ]; then
    echo -e "${YELLOW}Warning: This script is designed for Jetson devices${NC}"
    read -p "Continue anyway? (y/n) " -n 1 -r
    echo
    if [[ ! $REPLY =~ ^[Yy]$ ]]; then
        exit 1
    fi
fi

# Detect JetPack version
if [ -f /etc/nv_tegra_release ]; then
    JETPACK_VERSION=$(cat /etc/nv_tegra_release | head -n 1 | cut -d ' ' -f 2)
    echo -e "${GREEN}Detected JetPack version: $JETPACK_VERSION${NC}"
fi

# Check Ubuntu version
UBUNTU_VERSION=$(lsb_release -rs)
echo -e "${GREEN}Ubuntu version: $UBUNTU_VERSION${NC}"

if [ "$UBUNTU_VERSION" != "18.04" ]; then
    echo -e "${YELLOW}Warning: This script is optimized for Ubuntu 18.04${NC}"
    echo -e "${YELLOW}You may need to adjust ROS2 version and package names${NC}"
fi

echo ""
echo "Step 1: Updating system packages..."
sudo apt update
sudo apt upgrade -y

echo ""
echo "Step 2: Installing system dependencies..."
sudo apt install -y \
    curl \
    gnupg2 \
    lsb-release \
    software-properties-common \
    python3-pip \
    python3-dev \
    python3-setuptools \
    v4l-utils \
    libv4l-dev \
    libjpeg-dev \
    libpng-dev \
    libtiff-dev \
    libavcodec-dev \
    libavformat-dev \
    libswscale-dev \
    libgtk-3-dev \
    libcanberra-gtk-module \
    libcanberra-gtk3-module

echo ""
echo "Step 3: Installing ROS2 (if not already installed)..."
if [ "$UBUNTU_VERSION" = "18.04" ]; then
    echo -e "${YELLOW}Ubuntu 18.04 detected - ROS2 Foxy requires Ubuntu 20.04${NC}"
    echo "Options:"
    echo "  1. Use ROS2 Eloquent (officially supports Ubuntu 18.04)"
    echo "  2. Build ROS2 Foxy from source (time-consuming)"
    echo "  3. Upgrade to Ubuntu 20.04 (recommended for Foxy)"
    echo ""
    read -p "Install ROS2 Eloquent instead? (y/n) " -n 1 -r
    echo
    if [[ $REPLY =~ ^[Yy]$ ]]; then
        ROS_VERSION="eloquent"
        ROS_DISTRO="eloquent"
    else
        echo -e "${YELLOW}Attempting to install Foxy on Ubuntu 18.04 (may fail)${NC}"
        ROS_VERSION="foxy"
        ROS_DISTRO="foxy"
    fi
else
    ROS_VERSION="foxy"
    ROS_DISTRO="foxy"
fi

if ! dpkg -l | grep -q "ros-${ROS_DISTRO}-desktop"; then
    echo "ROS2 ${ROS_DISTRO^} not found. Installing..."
    
    # Set locale
    sudo locale-gen en_US en_US.UTF-8
    sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
    export LANG=en_US.UTF-8
    
    # Add ROS2 repository
    sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
    
    if [ "$UBUNTU_VERSION" = "18.04" ] && [ "$ROS_DISTRO" = "foxy" ]; then
        # For Foxy on 18.04, use 20.04 repository (may work)
        echo -e "${YELLOW}Using Ubuntu 20.04 repository for Foxy on 18.04${NC}"
        sudo sh -c 'echo "deb http://packages.ros.org/ros2/ubuntu focal main" > /etc/apt/sources.list.d/ros2-latest.list'
    else
        sudo sh -c 'echo "deb http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2-latest.list'
    fi
    
    sudo apt update
    
    # Try to install
    if sudo apt install -y "ros-${ROS_DISTRO}-desktop" python3-argcomplete 2>/dev/null; then
        echo -e "${GREEN}ROS2 ${ROS_DISTRO^} installed successfully${NC}"
    else
        echo -e "${RED}Failed to install ros-${ROS_DISTRO}-desktop${NC}"
        echo -e "${YELLOW}You may need to:${NC}"
        echo "  1. Build ROS2 from source (see: https://docs.ros.org/en/foxy/Installation/Ubuntu-Development-Setup.html)"
        echo "  2. Use ROS2 Eloquent instead (supports Ubuntu 18.04)"
        echo "  3. Upgrade to Ubuntu 20.04"
        echo ""
        read -p "Continue with manual ROS2 installation? (y/n) " -n 1 -r
        echo
        if [[ ! $REPLY =~ ^[Yy]$ ]]; then
            exit 1
        fi
    fi
    
    # Source ROS2 in current shell
    ROS_SETUP="/opt/ros/${ROS_DISTRO}/setup.bash"
    if ! grep -q "source $ROS_SETUP" ~/.bashrc; then
        echo "source $ROS_SETUP" >> ~/.bashrc
    fi
else
    echo -e "${GREEN}ROS2 already installed${NC}"
    # Detect which version is installed
    if dpkg -l | grep -q ros-foxy-desktop; then
        ROS_DISTRO="foxy"
    elif dpkg -l | grep -q ros-eloquent-desktop; then
        ROS_DISTRO="eloquent"
    fi
fi

echo ""
echo "Step 4: Installing ROS2 dependencies..."
ROS_SETUP="/opt/ros/${ROS_DISTRO}/setup.bash"
source "$ROS_SETUP" 2>/dev/null || {
    echo -e "${RED}Failed to source ROS2. Please install ROS2 first.${NC}"
    exit 1
}

sudo apt install -y \
    "ros-${ROS_DISTRO}-rclpy" \
    "ros-${ROS_DISTRO}-sensor-msgs" \
    "ros-${ROS_DISTRO}-std-msgs" \
    "ros-${ROS_DISTRO}-geometry-msgs" \
    "ros-${ROS_DISTRO}-cv-bridge" \
    "ros-${ROS_DISTRO}-image-transport" \
    "ros-${ROS_DISTRO}-camera-calibration" \
    python3-opencv \
    python3-numpy \
    python3-yaml

# Try to install vision_msgs (may not be available)
if apt-cache search "ros-${ROS_DISTRO}-vision-msgs" 2>/dev/null | grep -q vision-msgs; then
    sudo apt install -y "ros-${ROS_DISTRO}-vision-msgs" || true
    echo -e "${GREEN}vision_msgs installed${NC}"
else
    echo -e "${YELLOW}vision_msgs not available (will use custom messages)${NC}"
fi

echo ""
echo "Step 5: Installing Python dependencies..."
# Upgrade pip
python3 -m pip install --upgrade pip setuptools wheel

# Install NumPy (if not already installed via apt)
python3 -m pip install --upgrade numpy

# Install PyYAML
python3 -m pip install --upgrade pyyaml

echo ""
echo "Step 6: Installing PyTorch for Jetson..."
# Check for existing PyTorch
if python3 -c "import torch" 2>/dev/null; then
    echo -e "${GREEN}PyTorch already installed${NC}"
    python3 -c "import torch; print(f'PyTorch version: {torch.__version__}')"
else
    echo "Installing PyTorch for Jetson..."
    echo -e "${YELLOW}Note: This may take 10-20 minutes...${NC}"
    
    # Install PyTorch from NVIDIA's pre-built wheels
    # For JetPack 4.x (Ubuntu 18.04), use PyTorch 1.11+
    # Check: https://forums.developer.nvidia.com/t/pytorch-for-jetson/72048
    
    # Try to install from NVIDIA's repository
    if [ -f /etc/nv_tegra_release ]; then
        # For JetPack 4.6.1 (Ubuntu 18.04)
        # PyTorch 1.11.0 with CUDA 10.2
        echo "Installing PyTorch 1.11.0 for JetPack 4.6.1..."
        wget https://nvidia.box.com/shared/static/fjtbno0vpo676a25cgvuqc1wty0fkkg6.whl -O torch-1.11.0-cp36-cp36m-linux_aarch64.whl || \
        wget https://developer.download.nvidia.com/compute/redist/jp/v461/pytorch/torch-1.11.0-cp36-cp36m-linux_aarch64.whl || \
        echo -e "${YELLOW}PyTorch wheel download failed. Please install manually.${NC}"
        
        if [ -f torch-1.11.0-cp36-cp36m-linux_aarch64.whl ]; then
            python3 -m pip install torch-1.11.0-cp36-cp36m-linux_aarch64.whl
            rm torch-1.11.0-cp36-cp36m-linux_aarch64.whl
        fi
    else
        # Fallback: try pip install (may not work on Jetson)
        echo -e "${YELLOW}Attempting pip install of PyTorch (may fail on Jetson)${NC}"
        python3 -m pip install torch torchvision --index-url https://download.pytorch.org/whl/cpu || \
        echo -e "${RED}PyTorch installation failed. Please install manually from NVIDIA's repository.${NC}"
    fi
fi

echo ""
echo "Step 7: Installing Ultralytics (YOLO)..."
# Install ultralytics and dependencies
python3 -m pip install --upgrade \
    ultralytics \
    pillow \
    requests \
    tqdm \
    pandas \
    seaborn

echo ""
echo "Step 8: Installing additional CV dependencies..."
python3 -m pip install --upgrade \
    scipy \
    matplotlib

echo ""
echo "Step 9: Setting up camera permissions..."
# Add user to video group if not already
if ! groups $USER | grep -q video; then
    sudo usermod -a -G video $USER
    echo -e "${GREEN}Added user to video group${NC}"
    echo -e "${YELLOW}You may need to log out and back in for permissions to take effect${NC}"
else
    echo -e "${GREEN}User already in video group${NC}"
fi

echo ""
echo "Step 10: Configuring Jetson for optimal performance..."
# Set max power mode
if command -v nvpmodel &> /dev/null; then
    echo "Setting Jetson to max performance mode..."
    sudo nvpmodel -m 0 || true
    sudo jetson_clocks || true
    echo -e "${GREEN}Jetson performance mode configured${NC}"
fi

# Increase swap if needed (for memory-intensive operations)
SWAP_SIZE=$(free -m | grep Swap | awk '{print $2}')
if [ "$SWAP_SIZE" -lt 4096 ]; then
    echo -e "${YELLOW}Warning: Swap size is ${SWAP_SIZE}MB. Consider increasing to 4GB+ for YOLO inference.${NC}"
    echo "To increase swap, see: https://github.com/JetsonHacksNano/installSwapfile"
fi

echo ""
echo "Step 11: Verifying installations..."
echo "Checking Python packages..."
python3 << EOF
import sys
packages = {
    'cv2': 'opencv-python',
    'numpy': 'numpy',
    'yaml': 'pyyaml',
    'torch': 'torch',
    'ultralytics': 'ultralytics',
    'rclpy': 'rclpy'
}

missing = []
for module, name in packages.items():
    try:
        __import__(module)
        print(f"✓ {name}")
    except ImportError:
        print(f"✗ {name} - MISSING")
        missing.append(name)

if missing:
    print(f"\nMissing packages: {', '.join(missing)}")
    sys.exit(1)
else:
    print("\nAll Python packages installed successfully!")
    
    # Check PyTorch CUDA
    try:
        import torch
        if torch.cuda.is_available():
            print(f"✓ PyTorch CUDA available: {torch.cuda.get_device_name(0)}")
        else:
            print("⚠ PyTorch CUDA not available (CPU only)")
    except:
        pass
EOF

if [ $? -ne 0 ]; then
    echo -e "${RED}Some packages failed to import. Please check the errors above.${NC}"
    exit 1
fi

echo ""
echo "Checking ROS2 packages..."
ROS_SETUP="/opt/ros/${ROS_DISTRO}/setup.bash"
source "$ROS_SETUP" 2>/dev/null || true
ros2 pkg list | grep -E "(rclpy|sensor_msgs|cv_bridge|image_transport)" || echo -e "${YELLOW}Some ROS2 packages may not be listed${NC}"

echo ""
echo "=========================================="
echo -e "${GREEN}Installation Complete!${NC}"
echo "=========================================="
echo ""
echo "Next steps:"
echo "1. Log out and back in (or run: newgrp video)"
echo "2. Source ROS2: source /opt/ros/${ROS_DISTRO}/setup.bash"
echo "3. Build your workspace:"
echo "   cd ~/ros2_ws"
echo "   colcon build --packages-select wbk_vision wbk_yolo --symlink-install"
echo "   source install/setup.bash"
echo "4. Test camera:"
echo "   ros2 run wbk_vision usb_camera_node"
echo "5. Run YOLO node:"
echo "   ros2 launch wbk_yolo vision.launch.py model_path:=yolo11n.engine"
echo ""
echo -e "${YELLOW}Note: Using ROS2 ${ROS_DISTRO^}${NC}"
echo "For troubleshooting, see JETSON_DEPENDENCIES.md"
echo ""

