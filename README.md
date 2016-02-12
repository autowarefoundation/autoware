# Autoware

Integrated open-source software for urban autonomous driving. The following functions are supported:

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
- Moving Object Tracking
- Sensor Calibration
- Sensor Fusion
- Cloud-based Dynamic Maps
- Android Navigation
- Simulation
- Gamificated HMI

Autoware is protected by BSD License. Please use it on your own responsibility. For safe use, we provide a ROSBAG-based simulation method for those who do not own real autonomous vehicles. In case that you use Autoware with real autonomous vehicles, **please formulate safety measures and assessment of risk before field testing.**

## License

* New BSD License
    * See LICENSE

## Spec Recommendation

- # of CPU cores: 8
- RAM size: 32GB
- Storage size: 30GB

## Requirements

- ROS indigo(Ubuntu 14.04) or ROS jade(Ubuntu 15.04)
- OpenCV 2.4.8 or higher **NOTE: Autoware does not support OpenCV 3. Please use OpenCV 2**
- Qt 5.2.1 or higher
- CUDA(Optional)
- FlyCapture2(optional)
- Armadillo

**Please use checkout revision before 2015/OCT/21 if you use Autoware on ROS hydro or Ubuntu 13.04, 13.10.**

### Install dependencies for Ubuntu 14.04 indigo

```
% sudo apt-get install ros-indigo-desktop-full ros-indigo-nmea-msgs ros-indigo-nmea-navsat-driver ros-indigo-sound-play
% sudo apt-get install libnlopt-dev freeglut3-dev qtbase5-dev libqt5opengl5-dev libssh2-1-dev libarmadillo-dev libpcap-dev gksu
```

**NOTE: Please do not install ros-indigo-velodyne-pointcloud package. Please uninstall it if you already installed.**


### Install dependencies for Ubuntu 15.04 jade

```
% sudo apt-get install ros-jade-desktop-full ros-jade-nmea-msgs ros-jade-nmea-navsat-driver ros-jade-sound-play
% sudo apt-get install libnlopt-dev freeglut3-dev qt5-default libqt5opengl5-dev libssh2-1-dev libarmadillo-dev libpcap-dev gksu
```

## How to Build

```
$ cd $HOME
$ git clone https://github.com/CPFL/Autoware.git
$ cd ~/Autoware/ros/src
$ catkin_init_workspace
$ cd ../
$ ./catkin_make_release
```

## How to Start

```
$ cd $HOME/Autoware/ros
$ ./run
```

## For Developers

Be careful for changing files under `ros/src/sensing/drivers/lidar/packages/velodyne`. There is **subtree**.
Original repository is [here](https://github.com/CPFL/velodyne). If you change those files from this
repository, you must use **git subtree push**. (Please never change and push code if you don't understand
`git subtree` well).

## Documents

See Autoware/docs. As of Aug 25 2015, we provide only Japanese documents. English documents will be added shortly.

## Research Papers for Citation

1. S. Kato, E. Takeuchi, Y. Ishiguro, Y. Ninomiya, K. Takeda, and T. Hamada. "An Open Approach to Autonomous Vehicles", IEEE Micro, Vol. 35, No. 6, pp. 60-69, 2015. [![Link](http://online.qmags.com/MIC1115/default.aspx?sessionID=7CF18C36BF00A40746B87387B&cid=3230522&eid=19656&pg=62&mode=2#pg62&mode2)](http://online.qmags.com/MIC1115/default.aspx?sessionID=7CF18C36BF00A40746B87387B&cid=3230522&eid=19656&pg=62&mode=2#pg62&mode2)

## Instruction Videos

### Quick Start
[![Quick Start](http://img.youtube.com/vi/ztUtN3ZG6N8/0.jpg)](https://www.youtube.com/watch?v=ztUtN3ZG6N8)

### Localization with GNSS
[![Localization with GNSS](http://img.youtube.com/vi/5x3szHneHzM/0.jpg)](https://www.youtube.com/watch?v=5x3szHneHzM)

### Localization without GNSS
[![Localization without GNSS](http://img.youtube.com/vi/rbtdllALbCE/0.jpg)](https://www.youtube.com/watch?v=rbtdllALbCE)

### Mapping
[![Mapping](http://img.youtube.com/vi/hsX4HX_XBM4/0.jpg)](https://www.youtube.com/watch?v=hsX4HX_XBM4)

### Detection
[![Detection](http://img.youtube.com/vi/UcoYqGniIkE/0.jpg)](https://www.youtube.com/watch?v=UcoYqGniIkE)

### Planning
[![Planning](http://img.youtube.com/vi/QOrsC1P8nN0/0.jpg)](https://www.youtube.com/watch?v=QOrsC1P8nN0)

## Sample Data

[3D map of Moriyama in Nagoya](http://db3.ertl.jp/autoware/sample_data/sample_moriyama_data.tar.gz)

[ROSBAG data of Moriyama driving](http://db3.ertl.jp/autoware/sample_data/sample_moriyama_150324.tar.gz)

## Contact

Autoware Developers (<autoware@googlegroups.com>)
