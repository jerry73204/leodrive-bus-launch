# Leo Drive Bus-ODD Launch

Launch packages for the [Leo Drive Bus-ODD Dataset](https://autowarefoundation.github.io/autoware-documentation/main/datasets/) with Autoware.

## Dataset

The Leo Drive Bus-ODD dataset was recorded on an Isuzu bus platform with:

| Sensor   | Model                     | Quantity        |
|----------|---------------------------|-----------------|
| LiDAR    | Velodyne VLP16            | 1 (front)       |
| LiDAR    | Velodyne VLP32C           | 2 (left, right) |
| Camera   | Lucid Vision Triton 5.4MP | 3               |
| GNSS/INS | Applanix POS LV 120       | 1               |

**Size**: ~10.9GB

## Quick Start

```bash
# Clone with submodules
git clone --recursive https://github.com/your-org/leodrive-bus-ood-launch.git
cd leodrive-bus-ood-launch

# Full setup
just setup

# Or step by step:
just setup-python    # Install rosbags Python package
just build           # Build all ROS packages
just download        # Download dataset (~10.9GB)
just migrate-all     # Migrate rosbags to Autoware 1.5.0 format
```

## Playing Rosbags

```bash
# Source the workspace
source install/setup.bash

# Play a migrated rosbag
just play data/all-sensors-bag1_migrated

# Or manually
ros2 bag play data/all-sensors-bag1_migrated
```

## Packages

| Package                          | Description                               |
|----------------------------------|-------------------------------------------|
| `leodrive_bus_sensor_kit_launch` | Sensor kit configuration and launch files |
| `leodrive_bus_vehicle_launch`    | Vehicle description and parameters        |
| `applanix`                       | Applanix message definitions (submodule)  |

## Rosbag Migration

The original rosbags use deprecated `autoware_auto_*` message types. The migration script converts them to Autoware 1.5.0 format (`autoware_*_msgs`).

```bash
# Migrate a single bag
just migrate data/all-sensors-bag1_compressed data/all-sensors-bag1_migrated

# Migrate all bags
just migrate-all
```

## Topics

After migration, the rosbags contain:

| Topic                                                | Type                     | Description                    |
|------------------------------------------------------|--------------------------|--------------------------------|
| `/sensing/lidar/concatenated/pointcloud`             | PointCloud2              | Concatenated LiDAR point cloud |
| `/sensing/lidar/{front,left,right}/velodyne_packets` | VelodyneScan             | Raw Velodyne packets           |
| `/lucid_vision/camera_{0,1,2}/raw_image`             | Image                    | Camera images                  |
| `/lucid_vision/camera_{0,1,2}/camera_info`           | CameraInfo               | Camera intrinsics              |
| `/sensing/imu/imu_data`                              | Imu                      | IMU data                       |
| `/gnss/fix`                                          | NavSatFix                | GNSS position                  |
| `/lvx_client/gsof/ins_solution_49`                   | NavigationSolutionGsof49 | Applanix INS                   |
| `/tf`, `/tf_static`                                  | TFMessage                | Transforms                     |

## License

Apache License 2.0
