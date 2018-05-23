# Build and install procedure for 96Boards Dragonboard820c

This document provides instructions for installing Autoware on 96Boards
Dragonboard820c board.

## Table of Contents
- [1) Hardware Setup](#1-hardware-setup)
   - [1.1) Hardware Requirements](#11-hardware-requirements)
- [2) Software Setup](#2-software-setup)
   - [2.1) Installing Debian Stretch](#21-installing-debian-stretch)
   - [2.2) Package Dependencies](#22-package-dependencies)
   - [2.3) Enable ZRAM Swap Space](#23-enable-zram-swap-space)
- [3) Build and Install ROS Kinetic](#3-build-and-install-ros-kinetic)
- [4) Build and Install ROS Dependency packages](#4-build-and-install-ros-dependency-packages)
- [5) Autoware Installation](#5-autoware-installation)

## 1) Hardware Setup

### 1.1) Hardware Requirements

- [Dragonboard410c](https://www.96boards.org/product/dragonboard410c/)
- [96Boards Compliant Power Supply](http://www.96boards.org/product/power/)
- [Serial Adapater](https://www.96boards.org/product/uartserial/)

## 2) Software Setup

### 2.1) Installing Debian Stretch

- Download the latest boot image available for Dragonboard820c from [Linaro Releases](http://snapshots.linaro.org/96boards/dragonboard820c/linaro/debian/latest/boot-linaro-buster-dragonboard-820c-211.img.gz) and unzip it.

- Download the debian buster image available for Dragonboard820c from [Linaro Releases](http://snapshots.linaro.org/96boards/dragonboard820c/linaro/debian/latest/linaro-buster-alip-dragonboard-820c-211.img.gz).

- Download the debian stretch image available for Dragonboard410c from [Linaro Releases](http://releases.linaro.org/96boards/dragonboard410c/linaro/debian/17.04.1/linaro-stretch-alip-qcom-snapdragon-arm64-20170607-246.img.gz).

> Note: Since there is no debian stretch image available as a release build, we are going to hack
        Dragonboard410c's debian stretch image to work with Dragonboard820c.

- Uncompress and mount both images as /mnt/buster and /mnt/stretch.

- Copy the kernel modules from debian buster (/mnt/buster/lib/modules/) to debian stretch mount point (/mnt/stretch/lib/modules).

- Copy the firmwares from debian buster (/mnt/buster/lib/firmware/) to debian stretch mount point (/mnt/stretch/lib/firmware).

- Pack the debian stretch image into a raw image.

- Flash both boot and stretch images using fastboot command below:

```shell
$ sudo fastboot flash boot boot-linaro-buster-dragonboard-820c*.img
$ sudo fastboot flash rootfs stretch.raw
```
- Boot Dragonboard820c with the modified images and login using serial console.

- Establish WiFi connection by following the instructions available [here](https://github.com/96boards/documentation/blob/master/consumer/guides/wifi_commandline.md).

### 2.2) Package Dependencies

Install the below packages on Dragonboard820c:

```shell
$ sudo apt-get install libxmu-dev libxi-dev libnlopt-dev freeglut3-dev qtbase5-dev libqt5opengl5-dev libssh2-1-dev libarmadillo-dev libpcap-dev gksu libgl1-mesa-dev libglew-dev python-wxgtk3.0 software-properties-common libmosquitto-dev libyaml-cpp-dev python-flask python-requests dirmngr libtf2-eigen-dev libpcl-dev
```

### 2.3) Enable ZRAM Swap Space

Swap space is necessary for building both ROS and Autoware from source.
Hence, enable ZRAM swap space by following the instructions [here](https://github.com/96boards/documentation/blob/master/consumer/guides/zram_swapspace.md).

## 3) Build and Install ROS Kinetic

Build and install ROS Kinetic by following the instructions [here](http://wiki.ros.org/kinetic/Installation/Source).
Only change required is to replace the `rosdep install` command with below:

```shell
rosdep install --from-paths src --ignore-src --rosdistro kinetic -y --os=debian:stretch
```

After installing, export the `ROS_DISTRO` variable

```shell
$ export ROS_DISTRO=kinetic
```

## 4) Build and Install ROS Dependency packages

**Since we are not going to use CUDA, it should be disabled in Autoware**

Following ROS packages are required by Autoware. Hence, build and
install them from source.

- Tf2_eigen: https://github.com/ros/geometry2/tree/kinetic-devel/test_tf2 
- Pcl_msgs: https://github.com/ros-perception/pcl_msgs 
- Message_generation: https://github.com/ros/message_generation 
- Dynamic_reconfigure: https://github.com/ros/dynamic_reconfigure 
- Pcl_conversions: https://github.com/ros-perception/perception_pcl/tree/melodic-devel/pcl_conversions 
- Pcl_ros: https://github.com/ros-perception/perception_pcl/tree/melodic-devel/pcl_ros 
- Jsk_footstep_msgs: https://github.com/jsk-ros-pkg/jsk_common_msgs 
- Jsk_recognition_msgs: https://github.com/jsk-ros-pkg/jsk_recognition/tree/master/jsk_recognition_msgs 
- Nmea_msgs: https://github.com/ros-drivers/nmea_msgs 
- Sound_play: https://github.com/ros-drivers/audio_common 
- Grid_map_cv: https://github.com/ethz-asl/grid_map/tree/master/grid_map_cv 
- Grid_map_msgs: https://github.com/ethz-asl/grid_map/tree/master/grid_map_msgs 
- Grid_map_core: https://github.com/ethz-asl/grid_map/tree/master/grid_map_core 
- Grid_map_ros: https://github.com/ethz-asl/grid_map/tree/master/grid_map_ros 
- Camera_calibration_parsers: https://github.com/ros-perception/image_common/tree/hydro-devel/camera_calibration_parsers 
- Camera_info_manager: https://github.com/ros-perception/image_common/tree/hydro-devel/camera_info_manager 
- Jsk_hark_msgs: https://github.com/jsk-ros-pkg/jsk_common_msgs 
- Image_geometry: https://github.com/ros-perception/vision_opencv 
- Jsk_topic_tools: https://github.com/jsk-ros-pkg/jsk_common
> Note: Insert #include <boost/format.hpp> to src/standalone_complexed_nodelet.cpp
- Jsk_recognition_utils: https://github.com/jsk-ros-pkg/jsk_recognition/tree/master/jsk_recognition_utils
- People_msgs: https://github.com/wg-perception/people/tree/indigo-devel/people_msgs 
- View_controller_msgs: https://github.com/ros-visualization/view_controller_msgs 
- Jsk_gui_msgs: https://github.com/jsk-ros-pkg/jsk_common
- Jsk_rviz_plugins: https://github.com/jsk-ros-pkg/jsk_visualization/tree/master/jsk_rviz_plugins 
- Gps_common: https://github.com/swri-robotics/gps_umd/tree/master/gps_common 

## 5) Autoware Installation

Exclude following packages from Autoware by creating a `CATKIN_IGNORE` file
in the package directories:

- Yamaha's Golfcart Controller
- Nvidia Driveworks

```shell
$ source /opt/ros/kinetic/setup.bash
$ cd $HOME
$ git clone https://github.com/CPFL/Autoware.git
$ cd ~/Autoware/ros/src
$ catkin_init_workspace
$ cd ../
$ ./catkin_make_release
```
