#!/bin/bash
# Run Autoware logging simulation with Leo Drive Bus-ODD dataset
#
# Usage: ./scripts/sim-logging.sh [rosbag_path]
#
# This script:
# 1. Launches Autoware logging simulator with leodrive_bus packages
# 2. Plays the specified rosbag in a separate terminal
#
# Prerequisites:
# - Build packages: just build
# - Download data: just download

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"

# Default rosbag path
ROSBAG_PATH="${1:-$REPO_ROOT/data/all-sensors-bag1_migrated}"
MAP_PATH="${2:-$REPO_ROOT/data/istanbul-map}"

# Check if rosbag exists
if [[ ! -d "$ROSBAG_PATH" ]]; then
    echo "Error: Rosbag not found: $ROSBAG_PATH"
    echo ""
    echo "Available rosbags:"
    ls -d "$REPO_ROOT/data/"*_migrated 2>/dev/null || echo "  No migrated rosbags found. Run: just download && just migrate-all"
    exit 1
fi

# Check if map exists
if [[ ! -d "$MAP_PATH" ]]; then
    echo "Error: Map not found: $MAP_PATH"
    echo "Run: just download"
    exit 1
fi

# Check for pointcloud map
if [[ ! -f "$MAP_PATH/pointcloud_map.pcd" ]] && ! ls "$MAP_PATH"/*.pcd &>/dev/null; then
    echo "Error: No pointcloud map (.pcd) found in $MAP_PATH"
    echo "Run: just download"
    exit 1
fi

# Check for lanelet2 map
if [[ ! -f "$MAP_PATH/lanelet2_map.osm" ]]; then
    echo "Error: lanelet2_map.osm not found in $MAP_PATH"
    exit 1
fi

echo "=== Leo Drive Bus-ODD Logging Simulation ==="
echo "Rosbag: $ROSBAG_PATH"
echo "Map: $MAP_PATH"
echo ""

# Source environments
source /opt/ros/humble/setup.bash
source /opt/autoware/1.5.0/setup.bash
source "$REPO_ROOT/install/setup.bash"

echo "Launching Autoware logging simulator..."
echo "After RViz opens, run in another terminal:"
echo "  ros2 bag play $ROSBAG_PATH --clock"
echo ""

# Launch logging simulator
play_launch launch autoware_launch logging_simulator.launch.xml \
    map_path:="$MAP_PATH" \
    vehicle_model:=leodrive_bus_vehicle \
    sensor_model:=leodrive_bus_sensor_kit \
    rviz:=true \
    perception:=false \
    planning:=false \
    control:=false
