# Livox ROS Driver 2

Livox ROS Driver 2 is the 2nd-generation driver package used to connect LiDAR products produced by Livox.

This package is based on the original Livox ROS Driver 2 and has been modified for use with kait3.

## Modifications

The main modifications are:

* Made `lidar_frame_id` configurable through the JSON configuration file.
* Made `imu_frame_id` configurable through the JSON configuration file.
* Made `lidar_topic_name` configurable through the JSON configuration file.
* Made `imu_topic_name` configurable through the JSON configuration file.
* Added `kait3_MID360s_config.json` for driver configuration.
* Added `kait3_MID360s.launch.py` for launching the driver.

## Prerequisites

This package is based on the original Livox ROS Driver 2.

For the original driver, see the [original README](https://github.com/Livox-SDK/livox_ros_driver2/blob/master/README.md).

The modified version is currently developed and tested with:

* Ubuntu 24.04 LTS
* ROS 2 Jazzy
* Livox Mid-360S

## Build and Run

For the general build and run procedure, see the [Build & Run Livox ROS Driver 2](https://github.com/Livox-SDK/livox_ros_driver2#2-build--run-livox-ros-driver-2) section of the original README.

### Build

The package can be built using either `colcon` or the provided `build.sh` script.

#### Using colcon

```bash
cd ~/ros2_ws

colcon build --symlink-install --packages-select livox_ros_driver2
```

#### Using build.sh

```bash
cd ~/ros2_ws/src/livox_ros_driver2

./build.sh jazzy
```

### Run

Launch the Livox ROS Driver 2 for kait3:

```bash
ros2 launch livox_ros_driver2 kait3_MID360s.launch.py
```

The launch file provides the following launch arguments:

| Argument           | Default value                           | Description                                                                                     |
| ------------------ | --------------------------------------- | ----------------------------------------------------------------------------------------------- |
| `xfer_format`      | `0`                                     | Point cloud output format. `0`: PointCloud2 (`PointXYZRTL`), `1`: customized point cloud format |
| `multi_topic`      | `1`                                     | Topic configuration. `0`: all LiDARs share the same topic, `1`: one topic per LiDAR             |
| `publish_freq`     | `10.0`                                  | Point cloud publish frequency in Hz                                                             |
| `config_file`      | `config/kait3_MID360s_config.json`      | Path to the Livox configuration JSON file relative to the package share directory               |
| `rviz`             | `false`                                 | Whether to launch RViz2                                                                         |
| `rviz_config_file` | `config/kait3_display_point_cloud.rviz` | Path to the RViz configuration file relative to the package share directory                     |

For example, to launch RViz2:

```bash
ros2 launch livox_ros_driver2 kait3_MID360s.launch.py rviz:=true
```

To change the publish frequency:

```bash
ros2 launch livox_ros_driver2 kait3_MID360s.launch.py publish_freq:=20.0
```

To display all available launch arguments:

```bash
ros2 launch livox_ros_driver2 kait3_MID360s.launch.py --show-args
```

## LiDAR config

The following JSON file defines the network settings, ROS 2 topics, frame IDs, and extrinsic parameters for the LiDAR devices.

```jsonc
{
  "lidar_summary_info": {
    "lidar_type": 8                                 // LiDAR type
  },
  "Mid360s": {
    "lidar_net_info": {
      "cmd_data_port": 56100,                       // LiDAR command port
      "push_msg_port": 56200,                       // LiDAR message port
      "point_data_port": 56300,                     // LiDAR point cloud data port
      "imu_data_port": 56400,                       // LiDAR IMU data port
      "log_data_port": 56500                        // LiDAR log data port
    },
    "host_net_info": [
      {
        "host_ip": "192.168.1.xxx",                 // Host PC IP address
        "cmd_data_port": 56101,                     // Host command port
        "push_msg_port": 56201,                     // Host message port
        "point_data_port": 56301,                   // Host point cloud data port
        "imu_data_port": 56401,                     // Host IMU data port
        "log_data_port": 56501                      // Host log data port
      }
    ]
  },
  "lidar_configs": [
    {
      "ip": "192.168.1.1xx",                         // LiDAR IP address
      "pcl_data_type": 1,                            // Point cloud data type
      "pattern_mode": 0,                             // Scan pattern mode
      "lidar_topic_name": "/livox/front/pointcloud", // LiDAR point cloud topic
      "imu_topic_name": "/livox/front/imu",          // IMU topic
      "lidar_frame_id": "front_lidar_link",          // TF frame ID
      "imu_frame_id": "front_imu_link",              // TF frame ID
      "extrinsic_parameter": {            
        "roll": 0.0,                                 // Rotation around X-axis
        "pitch": 0.0,                                // Rotation around Y-axis
        "yaw": 0.0,                                  // Rotation around Z-axis
        "x": 0,                                      // Translation along X-axis
        "y": 0,                                      // Translation along Y-axis
        "z": 0                                       // Translation along Z-axis
      }
    },
    {
      "ip": "192.168.1.1xx",
      "pcl_data_type": 1,
      "pattern_mode": 0,
      "lidar_topic_name": "/livox/rear/pointcloud",
      "imu_topic_name": "/livox/rear/imu",
      "lidar_frame_id": "rear_lidar_link",
      "imu_frame_id": "rear_imu_link",
      "extrinsic_parameter": {
        "roll": 0.0,
        "pitch": 0.0,
        "yaw": 0.0,
        "x": 0,
        "y": 0,
        "z": 0
      }
    }
  ]
}
```
