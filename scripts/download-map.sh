#!/bin/bash
# Download Istanbul Open Dataset pointcloud map
#
# Note: This is the Istanbul Open Dataset map, which may be from a different
# route than the Bus-ODD rosbags. No lanelet2 map is available yet.
#
# Source: https://autowarefoundation.github.io/autoware-documentation/main/datasets/
#
# Usage: ./scripts/download-map.sh [output_dir]

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

OUTPUT_DIR="${1:-$REPO_ROOT/data/istanbul-map}"

# Google Drive folder ID for pointcloud maps
# https://drive.google.com/drive/folders/1_jiQod4lO6-V2NDEr3d-M3XF_Nqmc0Xf
GDRIVE_FOLDER_ID="1_jiQod4lO6-V2NDEr3d-M3XF_Nqmc0Xf"

# Check for gdown
if ! command -v gdown &> /dev/null; then
    echo "Error: gdown is not installed."
    echo "Install with: pip install gdown"
    exit 1
fi

echo "=== Istanbul Open Dataset Map Downloader ==="
echo "Source: Google Drive folder $GDRIVE_FOLDER_ID"
echo "Destination: $OUTPUT_DIR"
echo ""
echo "Note: This map is from the Istanbul Open Dataset."
echo "      No lanelet2_map.osm is available - you'll need to create one."
echo ""

mkdir -p "$OUTPUT_DIR"

echo "Downloading pointcloud maps..."
gdown --folder "$GDRIVE_FOLDER_ID" -O "$OUTPUT_DIR"

echo ""
echo "Download complete: $OUTPUT_DIR"

# Create symlink for pointcloud_map.pcd (Autoware expects this name)
if [[ ! -f "$OUTPUT_DIR/pointcloud_map.pcd" ]]; then
    # Use the subsampled version (smaller, faster to load)
    if [[ -f "$OUTPUT_DIR/route3_ss05.pcd" ]]; then
        ln -sf route3_ss05.pcd "$OUTPUT_DIR/pointcloud_map.pcd"
        echo "Created symlink: pointcloud_map.pcd -> route3_ss05.pcd"
    fi
fi

# Copy dummy lanelet2_map.osm if not present
if [[ ! -f "$OUTPUT_DIR/lanelet2_map.osm" ]]; then
    DUMMY_LANELET="$SCRIPT_DIR/../data/istanbul-map/lanelet2_map.osm"
    if [[ -f "$DUMMY_LANELET" ]]; then
        cp "$DUMMY_LANELET" "$OUTPUT_DIR/"
        echo "Copied dummy lanelet2_map.osm (minimal placeholder)"
    fi
fi

echo ""
echo "Contents:"
ls -lh "$OUTPUT_DIR"

echo ""
echo "Note: Using dummy lanelet2_map.osm. For planning functionality,"
echo "      create a proper map using Vector Map Builder."
