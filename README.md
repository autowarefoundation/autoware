<div align="center"><img src="docs/images/autoware_logo_1.png" width="400"/></div>

|Branch | Status |
|-------|--------|
|Master |[![Build Status](https://travis-ci.org/CPFL/Autoware.svg?branch=master)](https://travis-ci.org/CPFL/Autoware) |
|Develop|[![Build Status](https://travis-ci.org/CPFL/Autoware.svg?branch=develop)](https://travis-ci.org/CPFL/Autoware)|

# Autoware
Open-source software for urban autonomous driving, maintained by [Tier IV](http://www.tier4.jp). The following functions are supported:

- 3D Localization
- 3D Mapping
- Path Planning
- Path Following
- Accel/Brake/Steering Control
- Data Logging
- Car/Pedestrian/Object Detection
- Traffic Signal Detection
- Traffic Light Recognition
- Lane Detection
- Object Tracking
- Sensor Calibration
- Sensor Fusion
- Cloud-oriented Maps
- Connected Automation
- Smartphone Navigation
- Software Simulation
- Virtual Reality

Autoware is protected by the BSD License. Please use it at your own discretion. For safe use, we provide a ROSBAG-based simulation method for those who do not own real autonomous vehicles. If you plan to use Autoware with real autonomous vehicles, **please formulate safety measures and assessment of risk before field testing.**

## Manuals and Documents

Free manuals can be found at [https://github.com/CPFL/Autoware-Manuals](https://github.com/CPFL/Autoware-Manuals). You are encouraged to contribute to the maintenance of these manuals. Thank you for your cooperation!

If you have a question please check the [Wiki](https://github.com/CPFL/Autoware/wiki), and the [FAQ](https://github.com/CPFL/Autoware/wiki/FAQ), [FAQ (JP)](https://github.com/CPFL/Autoware/wiki/FAQ(JP)).

## License

* New BSD License
    * See LICENSE

## Recommended Minimum System Specifications

- Number of CPU cores: 8
- RAM size: 32GB
- Storage size: 30GB

## Requirements

- ROS indigo (Ubuntu 14.04) or ROS jade (Ubuntu 15.04) or ROS kinetic (Ubuntu 16.04)
- OpenCV 2.4.10 or higher
- Qt 5.2.1 or higher
- CUDA (optional)
- FlyCapture2 (optional)
- Armadillo (optional)

**Please use checkout a revision before 2015/OCT/21 if you want to use Autoware on ROS Hydro or Ubuntu 13.04, 13.10.**

### Install system dependencies for Ubuntu 14.04 Indigo

```
% sudo apt-get install -y  python-catkin-pkg python-rosdep python-wstool ros-$ROS_DISTRO-catkin
% sudo add-apt-repository ppa:mosquitto-dev/mosquitto-ppa
% sudo apt-get update
% sudo apt-get install libmosquitto-dev
```

**NOTE: Please do not install ros-indigo-velodyne-pointcloud package. If it is already installed, please uninstall.**

### Install system dependencies for Ubuntu 16.04 Kinetic
```
% sudo apt-get update
% sudo apt-get install -y python-catkin-pkg python-rosdep python-wstool ros-$ROS_DISTRO-catkin libmosquitto-dev
```

**NOTE: Following packages are not supported in ROS Kinetic.**
- gazebo
- orb slam
- dpm ocv

## How to Build

1. Clone the repository

```
$ cd $HOME
$ git clone https://github.com/CPFL/Autoware.git --recurse-submodules
```
or if you already have a copy of the repo, run `$ git submodule update --init --recursive`.

2. Initialize the workspace, let rosdep to install the missing dependencies and compile.
 
```
$ cd ~/Autoware/ros/src
$ catkin_init_workspace
$ cd ../
$ rosdep install -y --from-paths src --ignore-src --rosdistro $ROS_DISTRO
$ ./catkin_make_release
```

###Caffe based object detectors
CV based detectors RCNN and SSD nodes are not automatically built.

To build these nodes please follow the respective node's README
[SSD](ros/src/computing/perception/detection/packages/cv_tracker/nodes/ssd/README.md)
[RCNN](ros/src/computing/perception/detection/lib/image/librcnn/README.md)
[Yolo2](ros/src/computing/perception/detection/packages/cv_tracker/nodes/yolo2/README.md)
[Yolo3](ros/src/computing/perception/detection/packages/yolo3_detector/README.md)


## How to Start

```
$ cd $HOME/Autoware/ros
$ ./run
```

## For Developers

Be careful when changing files under `ros/src/sensing/drivers/lidar/packages/velodyne`. There is **subtree**.
The original repository is [here](https://github.com/CPFL/velodyne). If you change those files from this
repository, you must use **git subtree push**. (Please never change and push code if you don't understand
`git subtree` well).

GitFlow, the git branching model, is used in the Autoware repository.
- When adding new features, you can branch off your feature branch from `develop`.  
  You can use the following command.  
  `$ git checkout -b feature/[your_branch_name] develop`
- When you find bugs in `master`, you can branch off your hotfix branch from `master`.  
  You can use the following command.  
  `$ git checkout -b hotfix/[your_branch_name] master`

See also [branching_model](https://github.com/CPFL/Autoware/blob/master/docs/en/branching_model.md) for tips on Autoware development, including the coding style and branching model.

More details [here](http://nvie.com/posts/a-successful-git-branching-model/)

## Main Packages

### Localization
- ndt_localizer
- icp_localizer

### Detection
- lidar_tracker
- cv_tracker
- trafficlight_recognizer

### Mission (Global) Planning
- lane_planner
- way_planner
- freespace_planner

### Motion (Local) Planning
- astar_planner
- lattice_planner
- dp_planner

### Vehicle Control
- waypoint_follower
- waypoint_maker

## Research Papers for Citation

1. S. Kato, S. Tokunaga, Y. Maruyama, S. Maeda, M. Hirabayashi, Y. Kitsukawa, A. Monrroy, T. Ando, Y. Fujii, and T. Azumi,``Autoware on Board: Enabling Autonomous Vehicles with Embedded Systems,'' In Proceedings of the 9th ACM/IEEE International Conference on Cyber-Physical Systems (ICCPS2018),  Porto (aka Oporto), Portugal, Apr. 2018. 

2. S. Kato, E. Takeuchi, Y. Ishiguro, Y. Ninomiya, K. Takeda, and T. Hamada. "An Open Approach to Autonomous Vehicles", IEEE Micro, Vol. 35, No. 6, pp. 60-69, 2015. [![Link](http://online.qmags.com/MIC1115/default.aspx?sessionID=7CF18C36BF00A40746B87387B&cid=3230522&eid=19656&pg=62&mode=2#pg62&mode2)](http://online.qmags.com/MIC1115/default.aspx?sessionID=7CF18C36BF00A40746B87387B&cid=3230522&eid=19656&pg=62&mode=2#pg62&mode2)

## Demo Videos

### Public Road Demonstration
[![Public Road Demonstration](http://img.youtube.com/vi/5DaQBZvZwAI/mqdefault.jpg)](https://www.youtube.com/watch?v=5DaQBZvZwAI)

### Test Field Demonstration
[![Test Field Demonstration](http://img.youtube.com/vi/zujGfJcZCpQ/mqdefault.jpg)](https://www.youtube.com/watch?v=zujGfJcZCpQ)

## Instructional Videos

### Quick Start
[![Quick Start](http://img.youtube.com/vi/NDNcy0C-Has/mqdefault.jpg)](https://www.youtube.com/watch?v=NDNcy0C-Has)

### Loading Map Data
[![Loading Map Data](http://img.youtube.com/vi/OpvTeTaiXo4/mqdefault.jpg)](https://www.youtube.com/watch?v=OpvTeTaiXo4)

### Localization with GNSS
[![Localization with GNSS](http://img.youtube.com/vi/sul-osvg42A/mqdefault.jpg)](https://www.youtube.com/watch?v=sul-osvg42A)

### Localization without GNSS
[![Localization without GNSS](http://img.youtube.com/vi/ODlxMzGTJzw/mqdefault.jpg)](https://www.youtube.com/watch?v=ODlxMzGTJzw)

### Mapping
[![Mapping](http://img.youtube.com/vi/ss6Blrz23h8/mqdefault.jpg)](https://www.youtube.com/watch?v=ss6Blrz23h8)

### Detection with SSD
[![SSD](http://img.youtube.com/vi/EjamMJjkjBA/mqdefault.jpg)](https://youtu.be/EjamMJjkjBA)

### Detection with Yolo2
[![Yolo2](http://img.youtube.com/vi/gG_ojWOmDO0/mqdefault.jpg)](https://youtu.be/gG_ojWOmDO0)

### Detection with Yolo3
[![Yolo v3 Autoware](https://img.youtube.com/vi/pO4vM4ehI98/0.jpg)](https://www.youtube.com/watch?v=pO4vM4ehI98)

### Detection with DPM
[![DPM](http://img.youtube.com/vi/P_BFQNbudlg/mqdefault.jpg)](https://youtu.be/P_BFQNbudlg)

### Detection with Euclidean Clustering
[![Clustering](http://img.youtube.com/vi/Tma2DKMxt4Y/mqdefault.jpg)](https://youtu.be/Tma2DKMxt4Y)

### Traffic Light Recognition
[![Traffic Light Recognition](http://img.youtube.com/vi/KmOdBms9r2w/mqdefault.jpg)](https://youtu.be/KmOdBms9r2w)

### Planning with ROSBAG
[![Planning with ROSBAG](http://img.youtube.com/vi/B3UUKFM6Hqg/mqdefault.jpg)](https://www.youtube.com/watch?v=B3UUKFM6Hqg)

### Planning with wf_simulator
[![Planning with wf_simulator](http://img.youtube.com/vi/HwB2NKqj2yg/mqdefault.jpg)](https://www.youtube.com/watch?v=HwB2NKqj2yg)

### Planning with Hybrid State A*
[![Planning with wf_simulator](http://img.youtube.com/vi/1WiqAHZHj8U/mqdefault.jpg)](https://www.youtube.com/watch?v=1WiqAHZHj8U)

### Calibration Toolkit
[![Calibration Toolkit](http://img.youtube.com/vi/pfBmfgHf6zg/mqdefault.jpg)](https://www.youtube.com/watch?v=pfBmfgHf6zg)

See [https://github.com/CPFL/Autoware/wiki/Calibration(EN)](https://github.com/CPFL/Autoware/wiki/Calibration(EN))

### Camera-LiDAR Calibration
See [Autoware Camera-LiDAR Calibration](ros/src/sensing/fusion/packages/autoware_camera_lidar_calibrator/README.md)

### Multi-LiDAR Calibration
See [Autoware Multi-LiDAR Calibration](ros/src/sensing/fusion/packages/multi_lidar_calibrator/README.md)

### Data Processor for Bag File
[![Data Processor](http://img.youtube.com/vi/M38Obmy-3Ko/mqdefault.jpg)](https://youtu.be/M38Obmy-3Ko)

### Ftrace
[![Ftrace](http://img.youtube.com/vi/RoIqKgerDUw/mqdefault.jpg)](https://youtu.be/RoIqKgerDUw)

## Sample Data

[Moriyama demo](https://github.com/CPFL/Autoware/wiki/Demo)

[ROSBAG data for calibration test](http://db3.ertl.jp/autoware/sample_data/kotacho-calibration-sample_20160621.bag.bz2)

[ROSBAG data for IROS 2016](http://db3.ertl.jp/autoware/sample_data/iros2016_two_vehicle_data.tar.gz)

## ROSBAG STORE

You can download many ROSBAG files for research and development of self-driving technology using Autoware.
[https://rosbag.tier4.jp](https://rosbag.tier4.jp)

## Contact

Autoware Developers (<autoware@googlegroups.com>)

Autoware Developers Slack Team (https://autoware.herokuapp.com/)

To subscribe to the Autoware Developers mailing list,
- If you have a Google account, go to https://groups.google.com/d/forum/autoware, and click the **Apply to Join Group** button.
- If you don't have a Google account, send an email to autoware+subscribe@googlegroups.com.
