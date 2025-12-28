#!/bin/bash
# Convert YOLO .pt model to TensorRT .engine format for Jetson
# This provides better performance on Jetson hardware

set -e

# Colors
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m' # No Color

echo "=========================================="
echo "YOLO Model to TensorRT Engine Converter"
echo "=========================================="
echo ""

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
INPUT_MODEL="${1:-$SCRIPT_DIR/train200epochs_best.pt}"
OUTPUT_MODEL="${2:-$SCRIPT_DIR/train200epochs_best.engine}"

# Check if input model exists
if [ ! -f "$INPUT_MODEL" ]; then
    echo -e "${RED}Input model not found: $INPUT_MODEL${NC}"
    echo ""
    echo "Usage: $0 [input_model.pt] [output_model.engine]"
    echo "Example: $0 train200epochs_best.pt train200epochs_best.engine"
    exit 1
fi

echo "Input model: $INPUT_MODEL"
echo "Output model: $OUTPUT_MODEL"
echo ""

# Check if ultralytics is installed
if ! python3 -c "import ultralytics" 2>/dev/null; then
    echo -e "${RED}ultralytics not found. Attempting installation...${NC}"
    echo ""
    
    # Try to use the manual installation script if it exists
    MANUAL_SCRIPT="$(dirname "$0")/install_ultralytics_manual.sh"
    if [ -f "$MANUAL_SCRIPT" ]; then
        echo "Using manual installation script (installs from source)..."
        chmod +x "$MANUAL_SCRIPT"
        if "$MANUAL_SCRIPT"; then
            echo -e "${GREEN}✓ Ultralytics installed successfully from source!${NC}"
        else
            echo -e "${RED}Failed to install ultralytics via manual script${NC}"
            echo ""
            echo "Please install ultralytics manually first:"
            echo "  ./install_ultralytics_manual.sh"
            echo ""
            echo "Or try direct source installation:"
            echo "  pip3 install --no-binary :all: ultralytics --no-cache-dir"
            exit 1
        fi
    else
        # Fallback: try installation methods directly
        echo "Attempting to install ultralytics..."
        set +e  # Don't exit on error
        
        # Method 1: Install from source (most reliable for ARM64/Jetson)
        echo "Installing from source (recommended for ARM64/Jetson)..."
        echo -e "${YELLOW}This will compile from source and may take 10-20 minutes...${NC}"
        if python3 -m pip install --no-binary :all: ultralytics --no-cache-dir 2>&1; then
            echo -e "${GREEN}✓ Ultralytics installed from source!${NC}"
        elif python3 -m pip install git+https://github.com/ultralytics/ultralytics.git --no-cache-dir 2>&1; then
            echo -e "${GREEN}✓ Ultralytics installed from GitHub!${NC}"
        else
            echo -e "${RED}Failed to install ultralytics${NC}"
            echo ""
            echo "Please install ultralytics manually:"
            echo "  pip3 install --no-binary :all: ultralytics --no-cache-dir"
            echo ""
            echo "Or run: ./install_ultralytics_manual.sh"
            exit 1
        fi
        set -e
    fi
    
    # Verify installation
    if ! python3 -c "import ultralytics" 2>/dev/null; then
        echo -e "${RED}Ultralytics installation verification failed${NC}"
        echo "Please install manually and try again."
        exit 1
    fi
fi

# Check if TensorRT is available
if ! python3 -c "import tensorrt" 2>/dev/null; then
    echo -e "${YELLOW}TensorRT Python bindings not found${NC}"
    echo "Attempting conversion anyway (ultralytics will handle it)..."
fi

# Final check that ultralytics is available
if ! python3 -c "import ultralytics" 2>/dev/null; then
    echo -e "${RED}ERROR: Ultralytics is still not available after installation attempt${NC}"
    echo ""
    echo "Please install ultralytics manually:"
    echo "  cd $(dirname "$0")"
    echo "  ./install_ultralytics_manual.sh"
    echo ""
    echo "Or:"
    echo "  pip3 install --no-binary :all: ultralytics --no-cache-dir"
    exit 1
fi

echo -e "${GREEN}Ultralytics is available. Starting conversion...${NC}"
echo "This may take several minutes..."
echo ""

# Convert using ultralytics
python3 << EOF
from ultralytics import YOLO
import os

input_model = "$INPUT_MODEL"
output_model = "$OUTPUT_MODEL"

print(f"Loading model: {input_model}")
model = YOLO(input_model)

print(f"Exporting to TensorRT engine: {output_model}")
print("This may take 5-15 minutes depending on model size...")

try:
    # Export to TensorRT engine
    # device=0 uses GPU, half=True uses FP16 for better performance
    model.export(
        format='engine',
        imgsz=416,  # Match your config
        device=0,   # Use GPU
        half=True,  # FP16 precision (faster, slightly less accurate)
        simplify=True,
        workspace=4  # 4GB workspace
    )
    
    # The export creates a file with .engine extension
    # Move it to our desired location if needed
    exported_file = input_model.replace('.pt', '.engine')
    if os.path.exists(exported_file) and exported_file != output_model:
        import shutil
        shutil.move(exported_file, output_model)
        print(f"Model exported and moved to: {output_model}")
    else:
        print(f"Model exported to: {exported_file}")
        if exported_file != output_model:
            print(f"Note: Expected {output_model} but got {exported_file}")
    
    print(f"\n✓ Conversion complete!")
    print(f"Engine file: {output_model if os.path.exists(output_model) else exported_file}")
    
except Exception as e:
    print(f"\n✗ Conversion failed: {e}")
    print("\nTroubleshooting:")
    print("1. Make sure you're on a Jetson device with TensorRT")
    print("2. Check that CUDA/GPU is available")
    print("3. Try: export CUDA_VISIBLE_DEVICES=0")
    exit(1)
EOF

if [ $? -eq 0 ]; then
    echo ""
    echo -e "${GREEN}Conversion successful!${NC}"
    
    # Check if file was created
    if [ -f "$OUTPUT_MODEL" ]; then
        FILE_SIZE=$(du -h "$OUTPUT_MODEL" | cut -f1)
        echo "Engine file: $OUTPUT_MODEL ($FILE_SIZE)"
    else
        # Check for .engine file with same base name
        BASE_NAME=$(basename "$INPUT_MODEL" .pt)
        DIR_NAME=$(dirname "$INPUT_MODEL")
        POSSIBLE_ENGINE="$DIR_NAME/$BASE_NAME.engine"
        if [ -f "$POSSIBLE_ENGINE" ]; then
            echo "Engine file found at: $POSSIBLE_ENGINE"
            echo "Consider renaming to match expected name"
        fi
    fi
    
    echo ""
    echo "Next steps:"
    echo "1. Update scripts to use .engine file"
    echo "2. Run: ./run_yolo.sh"
else
    echo -e "${RED}Conversion failed. See errors above.${NC}"
    exit 1
fi

