# Run lidar_centerpoint and lidar_centerpoint-tiny simultaneously

This tutorial is for showing `centerpoint` and `centerpoint_tiny`models’ results simultaneously, making it easier to visualize and compare the performance.

## Setup Development Environment

Follow the steps in the Source Installation ([link](https://autowarefoundation.github.io/autoware-documentation/main/installation/autoware/source-installation/)) in Autoware doc.

If you fail to build autoware environment according to lack of memory, then it is recommended to build autoware sequentially.

Source the ROS 2 Galactic setup script.

```bash
source /opt/ros/galactic/setup.bash
```

Build the entire autoware repository.

```bash
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release --parallel-workers=1
```

Or you can use a constrained number of CPU to build only one package.

```bash
export MAKEFLAGS="-j 4" && MAKE_JOBS=4 colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release --parallel-workers 1 --packages-select PACKAGE_NAME
```

Source the package.

```bash
source install/setup.bash
```

## Data Preparation

### Using rosbag dataset

```bash
ros2 bag play /YOUR/ROSBAG/PATH/ --clock 100
```

Don't forget to add `clock` inorder to sync between two rviz display.

You can also use the sample rosbag provided by autoware [here](https://autowarefoundation.github.io/autoware-documentation/main/tutorials/ad-hoc-simulation/rosbag-replay-simulation/).

If you want to merge several rosbags into one, you can refer to [this tool](https://github.com/jerry73204/rosbag2-merge).

### Using realtime LiDAR dataset

Set up your Ethernet connection according to 1.1 - 1.3 in [this website](http://wiki.ros.org/velodyne/Tutorials/Getting%20Started%20with%20the%20Velodyne%20VLP16).

Download Velodyne ROS driver

```bash
git clone -b ros2 https://github.com/ros-drivers/velodyne.git
```

Source the ROS 2 Galactic setup script.

```bash
source /opt/ros/galactic/setup.bash
```

Compile Velodyne driver

```bash
cd velodyne
rosdep install -y --from-paths . --ignore-src --rosdistro $ROS_DISTRO
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
```

Edit the configuration file. Specify the LiDAR device IP address in `./velodyne_driver/config/VLP32C-velodyne_driver_node-params.yaml`

```yaml
velodyne_driver_node:
  ros__parameters:
    device_ip: 192.168.1.201 //change to your LiDAR device IP address
    gps_time: false
    time_offset: 0.0
    enabled: true
    read_once: false
    read_fast: false
    repeat_delay: 0.0
    frame_id: velodyne
    model: 32C
    rpm: 600.0
    port: 2368
```

Launch the velodyne driver.

```bash
# Terminal 1
ros2 launch velodyne_driver velodyne_driver_node-VLP32C-launch.py
```

Launch the velodyne_pointcloud.

```bash
# Terminal 2
ros2 launch velodyne_pointcloud velodyne_convert_node-VLP32C-launch.py
```

Point Cloud data will be available on topic `/velodyne_points`. You can check with `ros2 topic echo /velodyne_points`.

Check [this website](http://wiki.ros.org/velodyne/Tutorials/Getting%20Started%20with%20the%20Velodyne%20VLP16) if there is any unexpected issue.

## Launch file setting

Several fields to check in `centerpoint_vs_centerpoint-tiny.launch.xml` before running lidar centerpoint.

- `input/pointcloud` : set to the topic with input data you want to subscribe.
- `model_path` : set to the path of the model.
- `model_param_path` : set to the path of model's config file.

## Run CenterPoint and CenterPoint-tiny simultaneously

Run

```bash
ros2 launch lidar_centerpoint centerpoint_vs_centerpoint-tiny.launch.xml
```

Then you will see two rviz window show immediately. On the left is the result for lidar centerpoint tiny, and on the right is the result for lidar centerpoint.

![two rviz2 display centerpoint and centerpoint_tiny](https://i.imgur.com/YAYehrf.jpg)

## Troubleshooting

### Bounding Box blink on rviz

To avoid Bounding Boxs blinking on rviz, you can extend bbox marker lifetime.

Set `marker_ptr->lifetime` and `marker.lifetime` to a longer lifetime.

- `marker_ptr->lifetime` are in `PATH/autoware/src/universe/autoware.universe/common/autoware_auto_perception_rviz_plugin/src/object_detection/object_polygon_detail.cpp`
- `marker.lifetime` are in `PATH/autoware/src/universe/autoware.universe/common/tier4_autoware_utils/include/tier4_autoware_utils/ros/marker_helper.hpp`

Make sure to rebuild packages after any change.
