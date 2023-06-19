# 1. Hardware setup

As a first step, prepare your hardwares including Sensor and ECU.

## Sample hardware configuration

This following hardware configuration is used throughout this tutorial.

- ECU setup
  - x86-based ECU: ADLINK AVA-3510
  - Jetson-based ECU: ADLINK RQX-58G
- Sensor setup
  - Sample configuration 1
    - Camera: TIER IV Automotive HDR Camera C1 (x2)
    - LiDAR: HESAI AT128 (x1)
  - Sample configuration 2
    - Camera: TIER IV Automotive HDR Camera C1 (x2)
    - LiDAR: HESAI Pandar XT32 (x1)

### Connection diagram

The figure below depicts the connection diagram between sensors and ECUs for this tutorial.
This network configuration, including applying the IP addresses to the specific interface, will be automatically done during the steps in [2.Installation](./02_installation.md) page.

**NOTE: Internet connection is required for 2.Installation step.**

The next [2.Installation](./02_installation.md) step requires the internet connection for git clone and ML model download.
Please connect an Ethernet cable to the port indicated in the figure below for the internet connection.

![connection diagram of sample system](connection.drawio.svg "connection diagram of sample system")

### Sensor driver

Edge.Auto supports a variety of sensor types. The following repositories are used to make those sensors available in your ROS2 environment.
Please refer to the each repositories for more details.

- Camera driver
  - [tier4/tier4_automotive_hdr_camera](https://github.com/tier4/tier4_automotive_hdr_camera): Kernel driver for using TIER IV cameras with Video4Linux2 interface.
  - [tier4/ros2_v4l2_camera](https://github.com/tier4/ros2_v4l2_camera): ROS2 package for camera driver using Video4Linux2.
- LIDAR driver
  - [tier4/nebula](https://github.com/tier4/nebula): ROS2 package for unified ethernet-based LiDAR driver.
- Sensor synchronization
  - [tier4/sensor_trigger](https://github.com/tier4/sensor_trigger): ROS2 package for generating sensor trigger signals.

### Sensor/ECU synchronization

In this sample system, clock synchronization and timing synchronization between sensors and ECUs are realized to achieve highly accurate sensor fusion.
The figure below depicts the synchronization design between sensors and ECUs in this sample system.

For more details, please refer to the [tier4/sensor_trigger](https://github.com/tier4/sensor_trigger) repository.

![synchronization design of sample system](synchronization.drawio.svg "synchronization design of sample system")

## 1-1. x86-based ECU

Before proceeding with [2.Installation](./02_installation.md) step, install Ubuntu 22.04 to your x86-based ECU.

## 2-2. Jetson-based ECU

Before proceeding with [2.Installation](./02_installation.md) step, install NVIDIA L4T R32.6.1 (including Ubuntu 18.04) to your Jetson-based ECU.

**NOTE: BSP installation for ADLINK RQX-58G**

RQX-58G need to be properly configured according to the official quick start guide from ADLINK Technology, Inc.
Please see the [official document](https://www.adlinktech.com/Products/Download.ashx?type=MDownload&isQuickStart=yes&file=1783%5croscube-x-bsp-qsg-l4t-32.5.0-kernel-1.0.8.pdf) in detail.
To download the BSP image, please visit the ADLINK official page [here](https://www.adlinktech.com/Products/DownloadSoftware.aspx?lang=en&pdNo=1783&MainCategory=ROS2-Solution.aspx&kind=BS). (If you are accessing the site for the first time, you will be prompted to create an account.)

While TIER IV camera driver ([tier4/tier4_automotive_hdr_camera](https://github.com/tier4/tier4_automotive_hdr_camera)) is included in the RQX-58G BSP official image, you can also update it during the following setup process.
