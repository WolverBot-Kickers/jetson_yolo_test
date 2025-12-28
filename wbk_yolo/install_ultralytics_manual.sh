#!/bin/bash
# Manual ultralytics installation script for Jetson Nano (ARM64)
# Run this if the main install script fails to install ultralytics

set -e

# Colors
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m'

echo "=========================================="
echo "Manual Ultralytics Installation for Jetson"
echo "=========================================="
echo ""

# Check Python
if ! command -v python3 &> /dev/null; then
    echo -e "${RED}Python 3 not found!${NC}"
    exit 1
fi

PYTHON_VER=$(python3 --version)
echo "Python: $PYTHON_VER"

# Check pip
if ! python3 -m pip --version &> /dev/null; then
    echo -e "${RED}pip not found! Installing...${NC}"
    sudo apt install -y python3-pip
    python3 -m pip install --upgrade pip
fi

echo "pip: $(python3 -m pip --version)"
echo ""

# Install build dependencies
echo "Installing build dependencies..."
sudo apt install -y \
    python3-dev \
    build-essential \
    libffi-dev \
    libssl-dev \
    gcc \
    g++ \
    make

# Upgrade pip and tools
echo "Upgrading pip, setuptools, wheel..."
python3 -m pip install --upgrade pip setuptools wheel

# Install ultralytics dependencies first
echo ""
echo "Installing ultralytics dependencies..."
python3 -m pip install --upgrade \
    numpy \
    pillow \
    requests \
    tqdm \
    pandas \
    seaborn \
    scipy \
    matplotlib \
    pyyaml \
    opencv-python-headless || echo -e "${YELLOW}Some dependencies had issues${NC}"

echo ""
echo "Attempting ultralytics installation..."
echo "This may take 10-20 minutes as it compiles from source..."
echo ""

# Method 1: Install from source (most reliable for ARM64)
echo -e "${GREEN}Method 1: Installing from source (recommended for ARM64)...${NC}"
if python3 -m pip install --no-binary :all: ultralytics --no-cache-dir --verbose 2>&1 | tee /tmp/ultra_install.log; then
    echo -e "${GREEN}✓ Ultralytics installed from source!${NC}"
    python3 -c "import ultralytics; print(f'Version: {ultralytics.__version__}')"
    exit 0
fi

# Method 2: Install from GitHub
echo ""
echo -e "${YELLOW}Method 1 failed. Trying Method 2: Install from GitHub...${NC}"
if python3 -m pip install git+https://github.com/ultralytics/ultralytics.git --no-cache-dir --verbose 2>&1 | tee -a /tmp/ultra_install.log; then
    echo -e "${GREEN}✓ Ultralytics installed from GitHub!${NC}"
    python3 -c "import ultralytics; print(f'Version: {ultralytics.__version__}')"
    exit 0
fi

# Method 3: Try specific version
echo ""
echo -e "${YELLOW}Method 2 failed. Trying Method 3: Install specific version...${NC}"
if python3 -m pip install "ultralytics==8.0.196" --no-cache-dir --verbose 2>&1 | tee -a /tmp/ultra_install.log; then
    echo -e "${GREEN}✓ Ultralytics 8.0.196 installed!${NC}"
    python3 -c "import ultralytics; print(f'Version: {ultralytics.__version__}')"
    exit 0
fi

# Method 4: Try without binary dependencies
echo ""
echo -e "${YELLOW}Method 3 failed. Trying Method 4: Install with minimal dependencies...${NC}"
if python3 -m pip install ultralytics --no-deps --no-cache-dir 2>&1 | tee -a /tmp/ultra_install.log; then
    echo -e "${YELLOW}Installing missing dependencies...${NC}"
    python3 -m pip install torch torchvision --no-cache-dir || true
    python3 -c "import ultralytics; print(f'Version: {ultralytics.__version__}')" && exit 0
fi

echo ""
echo -e "${RED}All installation methods failed!${NC}"
echo ""
echo "Error log saved to: /tmp/ultra_install.log"
echo ""
echo "Troubleshooting:"
echo "1. Check error log: cat /tmp/ultra_install.log"
echo "2. Make sure PyTorch is installed: python3 -c 'import torch'"
echo "3. Check Python version: python3 --version (should be 3.6+)"
echo "4. Try installing PyTorch first, then ultralytics"
echo ""
echo "Manual PyTorch install for Jetson:"
echo "  See: https://forums.developer.nvidia.com/t/pytorch-for-jetson/72048"
echo ""

exit 1

