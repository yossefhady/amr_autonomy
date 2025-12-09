#!/bin/bash

# ==============================================================================
# AMR Project Native Installer
# ==============================================================================
# This script installs all ROS 2 Jazzy dependencies directly on your system.
# It fixes the "Missing Plugin", "Command not found", and "Message Filter" errors.
# ==============================================================================

set -e # Exit immediately if a command exits with a non-zero status.

GREEN='\033[0;32m'
BLUE='\033[0;34m'
RED='\033[0;31m'
NC='\033[0m' # No Color

echo -e "${BLUE}=== 🚀 Starting AMR Environment Setup ===${NC}"

# 1. Update System
echo -e "${GREEN}[1/5] Updating System Repositories...${NC}"
sudo apt update

# 2. Install Critical Dependencies
echo -e "${GREEN}[2/5] Installing ROS 2 Packages...${NC}"

# Core Navigation & SLAM
sudo apt install -y \
    ros-jazzy-navigation2 \
    ros-jazzy-nav2-bringup \
    ros-jazzy-nav2-common \
    ros-jazzy-nav2-msgs \
    ros-jazzy-nav2-simple-commander \
    ros-jazzy-slam-toolbox

# Planners (CRITICAL for your config)
# We install BOTH to ensure safety regardless of config choice
sudo apt install -y \
    ros-jazzy-nav2-smac-planner \
    ros-jazzy-nav2-navfn-planner

# Simulation & Bridge
sudo apt install -y \
    ros-jazzy-ros-gz \
    ros-jazzy-gz-ros2-control \

# Robot Description & Utils
sudo apt install -y \
    ros-jazzy-xacro \
    ros-jazzy-joint-state-publisher-gui \
    ros-jazzy-robot-state-publisher \
    ros-jazzy-teleop-twist-keyboard

# Middleware (Fixes Message Dropping / Lag)
sudo apt install -y \
    ros-jazzy-rmw-cyclonedds-cpp

# 3. Configure Environment (CycloneDDS)
echo -e "${GREEN}[3/5] Configuring Environment...${NC}"

# Check if RMW is already set in bashrc
if ! grep -q "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" ~/.bashrc; then
    echo "Exporting RMW_IMPLEMENTATION to ~/.bashrc..."
    echo "" >> ~/.bashrc
    echo "# AMR Navigation Middleware Fix" >> ~/.bashrc
    echo "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" >> ~/.bashrc
    echo "export RCUTILS_COLORIZED_OUTPUT=1" >> ~/.bashrc
    echo "Added CycloneDDS to .bashrc"
else
    echo "CycloneDDS is already configured in .bashrc"
fi

# 4. Clean & Build Workspace
echo -e "${GREEN}[4/5] Building Workspace...${NC}"

# Ensure we are in the root of the workspace
if [ -d "src" ]; then
    echo "Cleaning old build artifacts..."
    rm -rf build/ install/ log/
    
    echo "Building with symlink-install..."
    colcon build --symlink-install
else
    echo -e "${RED}⚠️  Error: 'src' directory not found!${NC}"
    echo "Please run this script from inside your 'ros2_ws' folder."
    exit 1
fi

# 5. Final Instructions
echo ""
echo -e "${GREEN}=============================================${NC}"
echo -e "${GREEN}✅ INSTALLATION COMPLETE${NC}"
echo -e "${GREEN}=============================================${NC}"
echo ""
echo "To apply the changes, run this command now:"
echo -e "${BLUE}  source ~/.bashrc && source install/setup.bash${BLUE}"
echo ""
echo "Then launch your robot:"
echo -e "${BLUE}  ros2 launch amr_navigation bringup_all.launch.py${BLUE}"
echo ""