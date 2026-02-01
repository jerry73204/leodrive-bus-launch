# Leo Drive Bus-ODD Dataset Launch
# Usage: just <recipe>

set shell := ["bash", "-cu"]

repo_root := justfile_directory()
script_dir := repo_root / "scripts"
install_dir := repo_root / "install"
data_dir := repo_root / "data"

# Pinned versions
rosbags_version := "0.11.0"

# Default recipe
default:
    @just --list

# Install Python dependencies (rosbags for migration)
setup-python:
    pip install rosbags=={{rosbags_version}}

# Build all ROS packages
build:
    #!/usr/bin/env bash
    set -e
    cd {{repo_root}}
    source /opt/ros/humble/setup.bash
    colcon build \
        --base-paths src \
        --symlink-install \
        --cmake-args -DCMAKE_BUILD_TYPE=Release
    echo ""
    echo "Built packages to: {{install_dir}}"
    echo "Source with: source {{install_dir}}/setup.bash"

# Build applanix_msgs package only
build-applanix:
    #!/usr/bin/env bash
    set -e
    cd {{repo_root}}
    source /opt/ros/humble/setup.bash
    colcon build \
        --base-paths src/applanix/applanix_msgs \
        --symlink-install \
        --cmake-args -DCMAKE_BUILD_TYPE=Release
    echo ""
    echo "Built applanix_msgs to: {{install_dir}}"
    echo "Source with: source {{install_dir}}/setup.bash"

# Clean build artifacts
clean:
    rm -rf {{repo_root}}/build {{repo_root}}/install {{repo_root}}/log

# Download the Leo Drive Bus-ODD dataset (~10.9GB)
download output_dir=data_dir:
    {{script_dir}}/download.sh {{output_dir}}

# Migrate rosbag to Autoware 1.5.0 format
migrate input_bag output_bag:
    {{script_dir}}/migrate-to-autoware15.py {{input_bag}} {{output_bag}}

# Migrate all downloaded rosbags to Autoware 1.5.0 format
migrate-all data_dir_arg=data_dir:
    #!/usr/bin/env bash
    set -e
    shopt -s nullglob

    bags=({{data_dir_arg}}/*_compressed)
    if [[ ${#bags[@]} -eq 0 ]]; then
        echo "No *_compressed rosbags found in {{data_dir_arg}}"
        exit 1
    fi

    echo "Found ${#bags[@]} rosbags to migrate"
    echo ""

    migrated=0
    skipped=0
    for bag in "${bags[@]}"; do
        output="${bag%_compressed}_migrated"
        name=$(basename "$bag")

        if [[ -d "$output" ]]; then
            echo "[SKIP] $name (already migrated)"
            skipped=$((skipped + 1))
        else
            echo "[MIGRATE] $name"
            {{script_dir}}/migrate-to-autoware15.py "$bag" "$output"
            migrated=$((migrated + 1))
            echo ""
        fi
    done

    echo "========================================"
    echo "Done: $migrated migrated, $skipped skipped"

# Full setup: install Python deps, build ROS packages, download dataset
setup:
    just setup-python
    just build
    just download
    just migrate-all

# Play a rosbag (use migrated bags for Autoware 1.5.0)
play bag_path:
    #!/usr/bin/env bash
    source /opt/ros/humble/setup.bash
    source {{install_dir}}/setup.bash 2>/dev/null || true
    ros2 bag play {{bag_path}}
