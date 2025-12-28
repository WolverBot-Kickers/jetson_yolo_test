#!/bin/bash
# Run script for YOLO Detection Node
# Run this in Terminal 2 (after camera is running)

set -e

# Colors
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m' # No Color

echo "=========================================="
echo "YOLO Detection Node Launcher"
echo "=========================================="
echo ""

# Get model path (default to TensorRT engine for best performance)
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
MODEL_PATH="${1:-$SCRIPT_DIR/train200epochs_best.engine}"

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
    colcon build --packages-select wbk_yolo --symlink-install
    source install/setup.bash
    cd -
fi

# Check if camera is running
echo "Checking for camera feed..."
if ! timeout 2 ros2 topic list | grep -q "/camera/image_raw"; then
    echo -e "${RED}Camera topic /camera/image_raw not found!${NC}"
    echo ""
    echo "Please start the camera node first:"
    echo "  Terminal 1: ./run_camera.sh"
    echo ""
    exit 1
fi
echo -e "${GREEN}Camera feed detected${NC}"

# Check if YOLO node is already running
if pgrep -f "yolo_node" > /dev/null; then
    echo -e "${YELLOW}YOLO node already running${NC}"
    read -p "Kill existing YOLO node? (y/n) " -n 1 -r
    echo
    if [[ $REPLY =~ ^[Yy]$ ]]; then
        pkill -f "yolo_node"
        pkill -f "field_mask_node"
        pkill -f "distance_node"
        sleep 1
    else
        echo "Exiting. YOLO node already running."
        exit 0
    fi
fi

# Check if model file exists
if [ ! -f "$MODEL_PATH" ]; then
    # Try .engine first, then .pt
    ENGINE_PATH="${MODEL_PATH%.*}.engine"
    PT_PATH="${MODEL_PATH%.*}.pt"
    
    if [ -f "$SCRIPT_DIR/train200epochs_best.engine" ] && [ "$MODEL_PATH" = "$SCRIPT_DIR/train200epochs_best.engine" ]; then
        MODEL_PATH="$SCRIPT_DIR/train200epochs_best.engine"
    elif [ -f "$SCRIPT_DIR/train200epochs_best.pt" ] && [ "$MODEL_PATH" = "$SCRIPT_DIR/train200epochs_best.engine" ]; then
        echo -e "${YELLOW}Engine file not found, but .pt file exists${NC}"
        echo "Run: ./convert_to_engine.sh to convert for better performance"
        MODEL_PATH="$SCRIPT_DIR/train200epochs_best.pt"
    elif [ -f "$HOME/ros2_ws/src/wbk_yolo/train200epochs_best.engine" ]; then
        MODEL_PATH="$HOME/ros2_ws/src/wbk_yolo/train200epochs_best.engine"
    elif [ -f "$HOME/ros2_ws/src/wbk_yolo/train200epochs_best.pt" ]; then
        echo -e "${YELLOW}Using .pt file. Convert to .engine for better performance${NC}"
        MODEL_PATH="$HOME/ros2_ws/src/wbk_yolo/train200epochs_best.pt"
    elif [ ! -f "$MODEL_PATH" ]; then
        echo -e "${YELLOW}Model file not found: $MODEL_PATH${NC}"
        echo ""
        echo "Usage: $0 [model_path]"
        echo "Example: $0 train200epochs_best.engine"
        echo "Or: $0 /path/to/model.pt (will auto-convert if needed)"
        echo ""
        read -p "Continue anyway? (y/n) " -n 1 -r
        echo
        if [[ ! $REPLY =~ ^[Yy]$ ]]; then
            exit 1
        fi
    fi
fi

echo ""
echo -e "${GREEN}Starting YOLO detection node...${NC}"
echo "Model: $MODEL_PATH"
echo "Subscribing to: /camera/image_raw"
echo "Publishing to:"
echo "  - /vision/ball_dets"
echo "  - /vision/goal_dets"
echo "  - /vision/robot_dets"
echo "  - /vision/landmarks"
echo "  - /vision/debug_image"
echo ""
echo "Press Ctrl+C to stop"
echo ""

# Run YOLO launch file
ros2 launch wbk_yolo vision.launch.py model_path:="$MODEL_PATH"

