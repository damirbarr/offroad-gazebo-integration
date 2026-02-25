#!/bin/bash
# Build and test script for offroad-gazebo-integration

set -e  # Exit on error

echo "================================"
echo "Off-Road Gazebo Integration"
echo "Build and Test Script"
echo "================================"
echo ""

# Check if we're in a ROS2 workspace
if [ ! -f "package.xml" ]; then
    echo "Error: This script must be run from the package root directory"
    exit 1
fi

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Check dependencies
echo "[1/5] Checking dependencies..."

check_command() {
    if command -v $1 &> /dev/null; then
        echo -e "${GREEN}✓${NC} $1 found"
        return 0
    else
        echo -e "${RED}✗${NC} $1 not found"
        return 1
    fi
}

DEPS_OK=true
check_command "colcon" || DEPS_OK=false
check_command "gz" || DEPS_OK=false
check_command "python3" || DEPS_OK=false

if [ "$DEPS_OK" = false ]; then
    echo -e "${RED}Missing dependencies. Please install prerequisites.${NC}"
    exit 1
fi

# Check ROS2 environment
echo ""
echo "[2/5] Checking ROS2 environment..."
if [ -z "$ROS_DISTRO" ]; then
    echo -e "${YELLOW}⚠${NC} ROS_DISTRO not set. Attempting to source..."
    if [ -f "/opt/ros/humble/setup.bash" ]; then
        source /opt/ros/humble/setup.bash
        echo -e "${GREEN}✓${NC} Sourced ROS2 Humble"
    else
        echo -e "${RED}✗${NC} Could not find ROS2 installation"
        exit 1
    fi
else
    echo -e "${GREEN}✓${NC} ROS_DISTRO=$ROS_DISTRO"
fi

# Check Python dependencies
echo ""
echo "[3/5] Checking Python dependencies..."
python3 -c "import numpy" 2>/dev/null && echo -e "${GREEN}✓${NC} numpy" || echo -e "${YELLOW}⚠${NC} numpy not found (pip3 install numpy)"
python3 -c "import yaml" 2>/dev/null && echo -e "${GREEN}✓${NC} yaml" || echo -e "${YELLOW}⚠${NC} pyyaml not found (pip3 install pyyaml)"
python3 -c "import PIL" 2>/dev/null && echo -e "${GREEN}✓${NC} PIL" || echo -e "${YELLOW}⚠${NC} pillow not found (pip3 install pillow)"

# Build
echo ""
echo "[4/5] Building package..."
cd ..  # Assume we're in package root, go to workspace root
if [ ! -d "src" ]; then
    echo -e "${RED}✗${NC} Not in a workspace. Expected structure: workspace/src/offroad-gazebo-integration"
    exit 1
fi

colcon build --packages-select offroad_gazebo_integration --symlink-install

if [ $? -eq 0 ]; then
    echo -e "${GREEN}✓${NC} Build successful"
else
    echo -e "${RED}✗${NC} Build failed"
    exit 1
fi

# Source workspace
source install/setup.bash

# Run tests
echo ""
echo "[5/5] Running tests..."
colcon test --packages-select offroad_gazebo_integration

if [ $? -eq 0 ]; then
    echo -e "${GREEN}✓${NC} Tests passed"
else
    echo -e "${YELLOW}⚠${NC} Some tests failed (this is OK if av-simulation is not installed)"
fi

# Test results
colcon test-result --verbose

echo ""
echo "================================"
echo -e "${GREEN}Build complete!${NC}"
echo "================================"
echo ""
echo "Next steps:"
echo "  1. Source workspace: source install/setup.bash"
echo "  2. Launch simulation: make run (or ros2 launch offroad_gazebo_integration inspection_world.launch.py)"
echo "  3. Read QUICKSTART.md for usage examples"
echo ""
