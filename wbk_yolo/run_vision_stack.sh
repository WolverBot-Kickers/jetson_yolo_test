#!/bin/bash
# Run script for Wolverbot Kickers Vision Stack
# Starts both camera and YOLO detection nodes in background
# 
# NOTE: For debugging, use separate terminals:
#   Terminal 1: ./run_camera.sh
#   Terminal 2: ./run_yolo.sh [model_path]

set -e

# Colors
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m' # No Color

echo "=========================================="
echo "Wolverbot Kickers Vision Stack Launcher"
echo "=========================================="
echo ""

# Check if ROS2 is sourced
if ! command -v ros2 &> /dev/null; then
    echo -e "${RED}ROS2 not found. Sourcing...${NC}"
    if [ -f /opt/ros/eloquent/setup.bash ]; then
        source /opt/ros/eloquent/setup.bash
    elif [ -f /opt/ros/foxy/setup.bash ]; then
        source /opt/ros/foxy/setup.bash
    else
        echo -e "${RED}ROS2 not installed. Please run install_jetson_yolo.sh first${NC}"
        exit 1
    fi
fi

# Check if workspace is sourced
if [ -z "$ROS_DISTRO" ]; then
    echo -e "${YELLOW}ROS_DISTRO not set. Sourcing workspace...${NC}"
    if [ -f ~/ros2_ws/install/setup.bash ]; then
        source ~/ros2_ws/install/setup.bash
    else
        echo -e "${YELLOW}Workspace not built. Building now...${NC}"
        cd ~/ros2_ws
        colcon build --packages-select wbk_vision wbk_yolo --symlink-install
        source install/setup.bash
        cd -
    fi
fi

# Get model path (default to TensorRT engine for best performance)
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
MODEL_PATH="${1:-$SCRIPT_DIR/train200epochs_best.engine}"

if [ ! -f "$MODEL_PATH" ]; then
    # Try .engine first, then .pt
    if [ -f "$SCRIPT_DIR/train200epochs_best.engine" ]; then
        MODEL_PATH="$SCRIPT_DIR/train200epochs_best.engine"
    elif [ -f "$SCRIPT_DIR/train200epochs_best.pt" ]; then
        echo -e "${YELLOW}Engine file not found, using .pt file${NC}"
        echo "Run: ./convert_to_engine.sh for better performance"
        MODEL_PATH="$SCRIPT_DIR/train200epochs_best.pt"
    elif [ -f "$HOME/ros2_ws/src/wbk_yolo/train200epochs_best.engine" ]; then
        MODEL_PATH="$HOME/ros2_ws/src/wbk_yolo/train200epochs_best.engine"
    elif [ -f "$HOME/ros2_ws/src/wbk_yolo/train200epochs_best.pt" ]; then
        MODEL_PATH="$HOME/ros2_ws/src/wbk_yolo/train200epochs_best.pt"
    elif [ ! -f "$MODEL_PATH" ]; then
        echo -e "${YELLOW}Model file not found: $MODEL_PATH${NC}"
        echo "Usage: $0 [model_path]"
        echo "Example: $0 train200epochs_best.engine"
        echo "Or: $0 /path/to/model.pt"
    read -p "Continue anyway? (y/n) " -n 1 -r
    echo
    if [[ ! $REPLY =~ ^[Yy]$ ]]; then
        exit 1
    fi
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
        echo "Using existing camera node"
        CAMERA_RUNNING=true
    fi
fi

# Check if YOLO node is already running
if pgrep -f "yolo_node" > /dev/null; then
    echo -e "${YELLOW}YOLO node already running${NC}"
    read -p "Kill existing YOLO node? (y/n) " -n 1 -r
    echo
    if [[ $REPLY =~ ^[Yy]$ ]]; then
        pkill -f "yolo_node"
        sleep 1
    else
        echo "Using existing YOLO node"
        YOLO_RUNNING=true
    fi
fi

# Function to cleanup on exit
cleanup() {
    echo ""
    echo -e "${YELLOW}Shutting down vision stack...${NC}"
    pkill -f "usb_camera_node" || true
    pkill -f "yolo_node" || true
    pkill -f "field_mask_node" || true
    pkill -f "distance_node" || true
    echo -e "${GREEN}Cleanup complete${NC}"
    exit 0
}

trap cleanup SIGINT SIGTERM

# Start camera node if not running
if [ "$CAMERA_RUNNING" != "true" ]; then
    echo -e "${GREEN}Starting camera node...${NC}"
    ros2 run wbk_vision usb_camera_node &
    CAMERA_PID=$!
    sleep 2
    
    # Check if camera started successfully
    if ! kill -0 $CAMERA_PID 2>/dev/null; then
        echo -e "${RED}Camera node failed to start${NC}"
        exit 1
    fi
    echo -e "${GREEN}Camera node started (PID: $CAMERA_PID)${NC}"
fi

# Wait for camera to be ready
echo "Waiting for camera to initialize..."
sleep 3

# Check if camera topic exists
if ! timeout 2 ros2 topic list | grep -q "/camera/image_raw"; then
    echo -e "${RED}Camera topic not found. Check camera connection.${NC}"
    cleanup
    exit 1
fi

# Start YOLO launch file
if [ "$YOLO_RUNNING" != "true" ]; then
    echo -e "${GREEN}Starting YOLO detection node...${NC}"
    echo "Model: $MODEL_PATH"
    ros2 launch wbk_yolo vision.launch.py model_path:="$MODEL_PATH" &
    YOLO_PID=$!
    sleep 2
    
    # Check if YOLO started successfully
    if ! kill -0 $YOLO_PID 2>/dev/null; then
        echo -e "${RED}YOLO node failed to start${NC}"
        cleanup
        exit 1
    fi
    echo -e "${GREEN}YOLO node started (PID: $YOLO_PID)${NC}"
fi

echo ""
echo "=========================================="
echo -e "${GREEN}Vision stack running!${NC}"
echo "=========================================="
echo ""
echo "Topics available:"
ros2 topic list | grep -E "(camera|vision)" || true
echo ""
echo "Press Ctrl+C to stop"
echo ""

# Wait for user interrupt
wait

