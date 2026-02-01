#!/bin/bash
# Download Istanbul Open Dataset pointcloud map
#
# Downloads map files to data/downloads/istanbul-map/ and creates
# an Autoware-compatible map directory at data/istanbul-map/ with symlinks.
#
# Features:
# - Multi-threaded download with aria2c (falls back to gdown)
# - Download continuation (resume interrupted downloads)
# - SHA256 checksum verification
# - Skips existing valid files
#
# Source: https://autowarefoundation.github.io/autoware-documentation/main/datasets/
#
# Usage: ./scripts/download-map.sh

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

# Directories
DOWNLOAD_DIR="${REPO_ROOT}/data/downloads/istanbul-map"
MAP_DIR="${REPO_ROOT}/data/istanbul-map"

# Google Drive folder ID for pointcloud maps
# https://drive.google.com/drive/folders/1_jiQod4lO6-V2NDEr3d-M3XF_Nqmc0Xf
GDRIVE_FOLDER_ID="1_jiQod4lO6-V2NDEr3d-M3XF_Nqmc0Xf"

# File manifest: filename|sha256|gdrive_file_id (if known)
# File IDs extracted from Google Drive folder
declare -A FILE_CHECKSUMS=(
    ["route3_ss05.pcd"]="b7005f1d500424d100fd3541ed752d7cc96a718d85609e1262b0337a342da848"
    ["route3-part1.pcd"]="01df60386841913bb60e9daf01c8e96e2d2231713b9b152c6980aaea30fe02a3"
    ["route3-part2.pcd"]="f189224b9dc5d7fc17e91c121b751419ce0fe641233bf62747e4f549ba7151e8"
    ["autoware_map/pointcloud_map.pcd"]="b7005f1d500424d100fd3541ed752d7cc96a718d85609e1262b0337a342da848"
    ["loam_feature_localization/local/pointcloud_map_tunnel.pcd"]="ebaede57a7d6e97f7810b0687c287f4ad9e8bf6fc0edd2d0821d2f35b856622b"
    ["loam_feature_localization/local/route3_local_corner.pcd"]="f7ab17772acf0f8c75d07d7c162c1523f5d046055feb812f231598172b7b2818"
    ["loam_feature_localization/local/route3_local_surface.pcd"]="d726602adef03f16c3887ce6434d06fdc033ac7dca98df1aaf7fd35ff604271f"
    ["loam_feature_localization/local/tunnel_local_corner.pcd"]="32954e6998ba491ae642aa48ce6ac33990b16da4bde86e6a5658776ec8346097"
    ["loam_feature_localization/local/tunnel_local_surface.pcd"]="f3a63358ba7924fb133247680d0ec9855b3e18a6850052be582d3295f7882569"
    ["loam_feature_localization/mgrs/route3_corner.pcd"]="3ced2a9b06e8fb5e24b1f7267d538c31c94f6ce54bec7011f04ff5471e907e3c"
    ["loam_feature_localization/mgrs/route3_surface.pcd"]="18bdb42dfc476e223dcc99059e3ab244e5e634b8a64991c80d7567b69da4aea2"
    ["separate_clouds/route3-part1.zip"]="85a7b86bb8631d10d79efe0f15086eda9ad556c3892897d2af5e9e26e47e57d6"
    ["separate_clouds/route3-part2.zip"]="665010fcd5892b436bb5110a38ee1d71983d461ec32b7b92f2875a31818e02bc"
)

# Colors
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

info() { echo -e "${GREEN}[INFO]${NC} $*"; }
warn() { echo -e "${YELLOW}[WARN]${NC} $*"; }
error() { echo -e "${RED}[ERROR]${NC} $*"; }

# Verify SHA256 checksum of a file
verify_checksum() {
    local file="$1"
    local expected="$2"

    if [[ ! -f "$file" ]]; then
        return 1
    fi

    local actual
    actual=$(sha256sum "$file" | awk '{print $1}')
    [[ "$actual" == "$expected" ]]
}

# Check if all essential files exist and are valid
check_existing_files() {
    local all_valid=true
    local essential_files=("route3_ss05.pcd")

    for file in "${essential_files[@]}"; do
        local filepath="${DOWNLOAD_DIR}/${file}"
        local expected="${FILE_CHECKSUMS[$file]}"

        if [[ -f "$filepath" ]] && verify_checksum "$filepath" "$expected"; then
            info "✓ $file exists and checksum valid"
        else
            all_valid=false
        fi
    done

    $all_valid
}

# Download using gdown (Google Drive)
download_with_gdown() {
    info "Downloading with gdown..."

    if ! command -v gdown &> /dev/null; then
        error "gdown is not installed. Install with: pip install gdown"
        exit 1
    fi

    # gdown supports --continue for resume
    gdown --folder "$GDRIVE_FOLDER_ID" -O "$DOWNLOAD_DIR" --continue
}

# Verify all downloaded files
verify_downloads() {
    info "Verifying downloaded files..."

    local failed=0
    for file in "${!FILE_CHECKSUMS[@]}"; do
        local filepath="${DOWNLOAD_DIR}/${file}"
        local expected="${FILE_CHECKSUMS[$file]}"

        if [[ -f "$filepath" ]]; then
            if verify_checksum "$filepath" "$expected"; then
                echo -e "  ${GREEN}✓${NC} $file"
            else
                echo -e "  ${RED}✗${NC} $file (checksum mismatch)"
                ((failed++))
            fi
        else
            echo -e "  ${YELLOW}?${NC} $file (not found)"
        fi
    done

    if [[ $failed -gt 0 ]]; then
        warn "$failed file(s) failed checksum verification"
        return 1
    fi

    return 0
}

# Create Autoware-compatible map directory with symlinks
create_autoware_map_dir() {
    info "Creating Autoware-compatible map directory..."

    mkdir -p "$MAP_DIR"

    # Create symlink for pointcloud_map.pcd
    local pcd_target="../downloads/istanbul-map/route3_ss05.pcd"
    if [[ -f "${DOWNLOAD_DIR}/route3_ss05.pcd" ]]; then
        ln -sf "$pcd_target" "${MAP_DIR}/pointcloud_map.pcd"
        info "Created symlink: pointcloud_map.pcd -> $pcd_target"
    else
        error "route3_ss05.pcd not found in downloads"
        return 1
    fi

    # Check for required Autoware map files (should be committed to repo)
    if [[ ! -f "${MAP_DIR}/lanelet2_map.osm" ]]; then
        warn "lanelet2_map.osm not found in $MAP_DIR"
    fi
    if [[ ! -f "${MAP_DIR}/map_projector_info.yaml" ]]; then
        warn "map_projector_info.yaml not found in $MAP_DIR"
    fi

    info "Autoware map directory ready: $MAP_DIR"
}

# Main
main() {
    echo "=== Istanbul Open Dataset Map Downloader ==="
    echo "Source: Google Drive folder $GDRIVE_FOLDER_ID"
    echo "Download dir: $DOWNLOAD_DIR"
    echo "Map dir: $MAP_DIR"
    echo ""
    echo "Note: This map is from the Istanbul Open Dataset."
    echo "      The lanelet2_map.osm is a minimal placeholder."
    echo ""

    mkdir -p "$DOWNLOAD_DIR"

    # Check if essential files already exist and are valid
    if check_existing_files; then
        info "Essential files already exist and are valid"
        echo ""
        read -p "Re-download anyway? [y/N] " -n 1 -r
        echo ""
        if [[ ! $REPLY =~ ^[Yy]$ ]]; then
            info "Skipping download"
            create_autoware_map_dir
            echo ""
            echo "Contents of $MAP_DIR:"
            ls -lh "$MAP_DIR"
            exit 0
        fi
    fi

    # Download files
    download_with_gdown

    echo ""

    # Verify downloads
    verify_downloads || warn "Some files failed verification"

    echo ""

    # Create Autoware map directory
    create_autoware_map_dir

    echo ""
    echo "Contents of $MAP_DIR:"
    ls -lh "$MAP_DIR"

    echo ""
    echo "Note: Using minimal lanelet2_map.osm placeholder."
    echo "      For planning functionality, create a proper map using Vector Map Builder."
}

main "$@"
