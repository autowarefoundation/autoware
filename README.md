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

Autoware is protected by the BSD License. Please use it on at your own discretion. For safe use, we provide a ROSBAG-based simulation method for those who do not own real autonomous vehicles. In case you use Autoware with real autonomous vehicles, **please formulate safety measures and assessment of risk before field testing.**

## Manuals and Documents

Free manuals can be found at [https://github.com/CPFL/Autoware-Manuals](https://github.com/CPFL/Autoware-Manuals). You are encouraged to join maintenance of those manuals. Thanks for your cooperation!

See also Autoware/docs for the tips of Autoware development, including the coding style and branching model.

## License

* New BSD License
    * See LICENSE

## Spec Recommendation

- Number of CPU cores: 8
- RAM size: 32GB
- Storage size: 30GB

## Requirements

- ROS indigo (Ubuntu 14.04) or ROS jade (Ubuntu 15.04) or ROS kinetic (Ubuntu 16.04)
- OpenCV 2.4.10 or higher
- Qt 5.2.1 or higher
- CUDA(Optional)
- FlyCapture2 (Optional)
- Armadillo (Optional)

**Please use checkout revision before 2015/OCT/21 if you use Autoware on ROS hydro or Ubuntu 13.04, 13.10.**

### Install dependencies for Ubuntu 14.04 indigo

```
% sudo apt-get install ros-indigo-desktop-full ros-indigo-nmea-msgs ros-indigo-nmea-navsat-driver ros-indigo-sound-play ros-indigo-jsk-visualization ros-indigo-grid-map ros-indigo-gps-common
% sudo apt-get install ros-indigo-controller-manager ros-indigo-ros-control ros-indigo-ros-controllers ros-indigo-gazebo-ros-control ros-indigo-sicktoolbox ros-indigo-sicktoolbox-wrapper ros-indigo-joystick-drivers ros-indigo-novatel-span-driver
% sudo apt-get install libnlopt-dev freeglut3-dev qtbase5-dev libqt5opengl5-dev libssh2-1-dev libarmadillo-dev libpcap-dev gksu libgl1-mesa-dev libglew-dev
```

**NOTE: Please do not install ros-indigo-velodyne-pointcloud package. Please uninstall it if you already installed.**

### Install dependencies for Ubuntu 16.04 kinetic
```
% sudo apt-get install ros-kinetic-desktop-full ros-kinetic-nmea-msgs ros-kinetic-nmea-navsat-driver ros-kinetic-sound-play ros-kinetic-jsk-visualization ros-kinetic-grid-map ros-kinetic-gps-common
% sudo apt-get install ros-kinetic-controller-manager ros-kinetic-ros-control ros-kinetic-ros-controllers ros-kinetic-gazebo-ros-control ros-kinetic-joystick-drivers
% sudo apt-get install libnlopt-dev freeglut3-dev qtbase5-dev libqt5opengl5-dev libssh2-1-dev libarmadillo-dev libpcap-dev gksu libgl1-mesa-dev libglew-dev python-wxgtk3.0
```

**NOTE: Following packages are not supported in ROS Kinetic.**
- gazebo
- orb slam
- dpm ocv

## How to Build

```
$ cd $HOME
$ git clone https://github.com/CPFL/Autoware.git
$ cd ~/Autoware/ros/src
$ catkin_init_workspace
$ cd ../
$ ./catkin_make_release
```
###Caffe based object detectors
CV based detectors RCNN and SSD nodes are not automatically built.

To build these nodes please follow the respective node's README
[SSD](ros/src/computing/perception/detection/packages/cv_tracker/nodes/ssd/README.md)
[RCNN](ros/src/computing/perception/detection/lib/image/librcnn/README.md)
[Yolo2](ros/src/computing/perception/detection/packages/cv_tracker/nodes/yolo2/README.md)


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
- When you adding new features, you can branch off your feature branch from `develop`.  
  you can use the following command.  
  `$ git checkout -b feature/[your_branch_name] develop`
- When you find bugs in `master`, you can branch off your hotfix branch from `master`.  
  you can use the following command.  
  `$ git checkout -b hotfix/[your_branch_name] master`

See [docs/en/branching_model.md](docs/en/branching_model.md)

More details [here](http://nvie.com/posts/a-successful-git-branching-model/)

## Main Packages

### Localization
- ndt_localizer
- icp_localizer

### Detection
- lidar_tracker
- cv_tracker
- road_wizard

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

1. S. Kato, E. Takeuchi, Y. Ishiguro, Y. Ninomiya, K. Takeda, and T. Hamada. "An Open Approach to Autonomous Vehicles", IEEE Micro, Vol. 35, No. 6, pp. 60-69, 2015. [![Link](http://online.qmags.com/MIC1115/default.aspx?sessionID=7CF18C36BF00A40746B87387B&cid=3230522&eid=19656&pg=62&mode=2#pg62&mode2)](http://online.qmags.com/MIC1115/default.aspx?sessionID=7CF18C36BF00A40746B87387B&cid=3230522&eid=19656&pg=62&mode=2#pg62&mode2)

## Demo Videos

### Public Road Demonstration
[![Public Road Demonstration](http://img.youtube.com/vi/5DaQBZvZwAI/mqdefault.jpg)](https://www.youtube.com/watch?v=5DaQBZvZwAI)

### Test Field Demonstration
[![Test Field Demonstration](http://img.youtube.com/vi/zujGfJcZCpQ/mqdefault.jpg)](https://www.youtube.com/watch?v=zujGfJcZCpQ)

## Instruction Videos

### Quick Start
[![Quick Start](http://img.youtube.com/vi/m-4U84K7lvg/mqdefault.jpg)](https://www.youtube.com/watch?v=m-4U84K7lvg)

### Loading Map Data
[![Loading Map Data](http://img.youtube.com/vi/EJa4PHnjdRY/mqdefault.jpg)](https://www.youtube.com/watch?v=EJa4PHnjdRY)

### Localization with GNSS
[![Localization with GNSS](http://img.youtube.com/vi/5bj7gkFlul0/mqdefault.jpg)](https://www.youtube.com/watch?v=5bj7gkFlul0)

### Localization without GNSS
[![Localization without GNSS](http://img.youtube.com/vi/ODlxMzGTJzw/mqdefault.jpg)](https://www.youtube.com/watch?v=ODlxMzGTJzw)

### Mapping
[![Mapping](http://img.youtube.com/vi/HlQ0ohxvlgA/mqdefault.jpg)](https://www.youtube.com/watch?v=HlQ0ohxvlgA)

### Detection with SSD
[![SSD](http://img.youtube.com/vi/EjamMJjkjBA/mqdefault.jpg)](https://youtu.be/EjamMJjkjBA)

### Detection with Yolo2
[![Yolo2](http://img.youtube.com/vi/gG_ojWOmDO0/mqdefault.jpg)](https://youtu.be/gG_ojWOmDO0)

### Detection with DPM
[![DPM](http://img.youtube.com/vi/P_BFQNbudlg/mqdefault.jpg)](https://youtu.be/P_BFQNbudlg)

### Detection with Euclidean Clustering
[![Clustering](http://img.youtube.com/vi/Tma2DKMxt4Y/mqdefault.jpg)](https://youtu.be/Tma2DKMxt4Y)

### Traffic Light Recognition
[![Traffic Light Recognition](http://img.youtube.com/vi/KmOdBms9r2w/mqdefault.jpg)](https://youtu.be/KmOdBms9r2w)

### Planning with ROSBAG
[![Planning with ROSBAG](http://img.youtube.com/vi/LZTCDbcjIdw/mqdefault.jpg)](https://www.youtube.com/watch?v=LZTCDbcjIdw)

### Planning with wf_simulator
[![Planning with wf_simulator](http://img.youtube.com/vi/HwB2NKqj2yg/mqdefault.jpg)](https://www.youtube.com/watch?v=HwB2NKqj2yg)

### Planning with Hybrid State A*
[![Planning with wf_simulator](http://img.youtube.com/vi/1WiqAHZHj8U/mqdefault.jpg)](https://www.youtube.com/watch?v=1WiqAHZHj8U)

### Calibration Toolkit
[![Calibration Toolkit](http://img.youtube.com/vi/pfBmfgHf6zg/mqdefault.jpg)](https://www.youtube.com/watch?v=pfBmfgHf6zg)

### Data Processor for Bag File
[![Data Processor](http://img.youtube.com/vi/M38Obmy-3Ko/mqdefault.jpg)](https://youtu.be/M38Obmy-3Ko)

### Ftrace
[![Ftrace](http://img.youtube.com/vi/RoIqKgerDUw/mqdefault.jpg)](https://youtu.be/RoIqKgerDUw)

## Sample Data

[3D map of Moriyama in Nagoya](http://db3.ertl.jp/autoware/sample_data/sample_moriyama_data.tar.gz)

[ROSBAG data of Moriyama driving](http://db3.ertl.jp/autoware/sample_data/sample_moriyama_150324.tar.gz)

[Script for generating demo launch files of Moriyama](http://db3.ertl.jp/autoware/sample_data/my_launch.sh)

[ROSBAG data for Calibration](http://db3.ertl.jp/autoware/sample_data/kotacho-calibration-sample_20160621.bag.bz2)

## IROS 2016 Data

[ROSBAG data of Nagoya driving](http://db3.ertl.jp/autoware/sample_data/iros2016_two_vehicle_data.tar.gz)

## Contact

Autoware Developers (<autoware@googlegroups.com>)

Autoware Developers Slack Team (https://autoware.herokuapp.com/)

To subscribe to the Autoware Developers mailing list,
- If you have a Google account, go to https://groups.google.com/d/forum/autoware, and click the **Apply to Join Group** button.
- If you don't have a Google account, send an email to autoware+subscribe@googlegroups.com.
