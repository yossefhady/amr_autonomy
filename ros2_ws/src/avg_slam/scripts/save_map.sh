#!/bin/bash
# Map Saving Helper Script for AVG SLAM
# This script provides easy commands to save maps created during SLAM

set -e

# Colors
GREEN='\033[0;32m'
BLUE='\033[0;34m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

echo "=================================================="
echo "🗺️  AVG SLAM Map Saving Utility"
echo "=================================================="
echo ""

# Default output directory
DEFAULT_DIR="/home/yossef/avg_autonomy/ros2_ws/src/avg_navigation/maps"
MAP_NAME="${1:-my_warehouse_map}"
OUTPUT_DIR="${2:-$DEFAULT_DIR}"

# Create directory if it doesn't exist
echo -e "${BLUE}📁 Creating maps directory...${NC}"
mkdir -p "$OUTPUT_DIR"
echo -e "${GREEN}✓ Directory ready: $OUTPUT_DIR${NC}"
echo ""

# Full path for map
FULL_PATH="$OUTPUT_DIR/$MAP_NAME"

echo "=================================================="
echo "Choose a map saving method:"
echo "=================================================="
echo ""
echo "1. SLAM Toolbox Format (for localization later)"
echo "   - Saves as .data and .posegraph files"
echo "   - Best for continuing SLAM or localization"
echo ""
echo "2. Nav2 Map Server Format (PNG/YAML)"
echo "   - Saves as .pgm/.png and .yaml files"
echo "   - Best for navigation with Nav2"
echo ""
echo "3. Both Formats (Recommended)"
echo "   - Saves in both formats"
echo ""
echo -n "Enter choice (1, 2, or 3): "
read -r choice

echo ""
echo "=================================================="

case $choice in
    1)
        echo -e "${BLUE}💾 Saving map in SLAM Toolbox format...${NC}"
        echo ""
        echo "Running command:"
        echo "ros2 service call /slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph \"{filename: '$FULL_PATH'}\""
        echo ""
        
        ros2 service call /slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph "{filename: '$FULL_PATH'}"
        
        echo ""
        echo -e "${GREEN}✓ Map saved successfully!${NC}"
        echo "Files created:"
        echo "  - $FULL_PATH.data"
        echo "  - $FULL_PATH.posegraph"
        ;;
        
    2)
        echo -e "${BLUE}💾 Saving map in Nav2 format...${NC}"
        echo ""
        echo "Running command:"
        echo "ros2 run nav2_map_server map_saver_cli -f $FULL_PATH"
        echo ""
        
        ros2 run nav2_map_server map_saver_cli -f "$FULL_PATH"
        
        echo ""
        echo -e "${GREEN}✓ Map saved successfully!${NC}"
        echo "Files created:"
        echo "  - $FULL_PATH.pgm (or .png)"
        echo "  - $FULL_PATH.yaml"
        ;;
        
    3)
        echo -e "${BLUE}💾 Saving map in both formats...${NC}"
        echo ""
        
        # SLAM Toolbox format
        echo "1/2: Saving SLAM Toolbox format..."
        ros2 service call /slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph "{filename: '$FULL_PATH'}" > /dev/null 2>&1
        echo -e "${GREEN}✓ SLAM Toolbox format saved${NC}"
        
        # Nav2 format
        echo "2/2: Saving Nav2 format..."
        ros2 run nav2_map_server map_saver_cli -f "$FULL_PATH" > /dev/null 2>&1
        echo -e "${GREEN}✓ Nav2 format saved${NC}"
        
        echo ""
        echo -e "${GREEN}✓ Map saved in both formats successfully!${NC}"
        echo "Files created:"
        echo "  - $FULL_PATH.data"
        echo "  - $FULL_PATH.posegraph"
        echo "  - $FULL_PATH.pgm (or .png)"
        echo "  - $FULL_PATH.yaml"
        ;;
        
    *)
        echo -e "${YELLOW}❌ Invalid choice. Exiting.${NC}"
        exit 1
        ;;
esac

echo ""
echo "=================================================="
echo -e "${GREEN}📍 Map Location:${NC} $OUTPUT_DIR"
echo "=================================================="
echo ""
echo "Next steps:"
echo ""
echo "1. View the saved map:"
echo "   eog $FULL_PATH.pgm  # (if PNG/YAML format was saved)"
echo ""
echo "2. Use map for localization:"
echo "   - Edit slam_params.yaml and set mode: localization"
echo "   - Uncomment map_file_name: $FULL_PATH"
echo ""
echo "3. Use map for navigation:"
echo "   ros2 launch avg_navigation navigation.launch.py map:=$FULL_PATH.yaml"
echo ""
