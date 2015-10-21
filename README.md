# Autoware

Open-source software for urban autonomous driving. The following functions are supported:

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

## License

* New BSD License
    * See LICENSE

## Requirements

- ROS indigo(Ubuntu 13.10, 14.04) or ROS jade(Ubuntu 15.04)
- OpenCV 2.4.8 or higher(Autoware does not support OpenCV 3. Please use OpenCV 2.x.y)
- Qt 5.2.1 or higher
- CUDA(Optional)
- FlyCapture2(optional)
- Armadillo

### Install dependencies for Ubuntu 15.04 jade

```
% sudo apt-get install ros-jade-desktop-full ros-jade-nmea-msgs ros-jade-nmea-navsat-driver ros-jade-sound-play
% sudo apt-get install libnlopt-dev freeglut3-dev qt5-default libqt5opengl5-dev libssh2-1-dev libarmadillo-dev
```

**NOTE** The install directory of Qt5 headers on Ubuntu 15.04 is differenct from Ubuntu 14.04, we need to
create symbolic for adding directory path same as Ubuntu 14.04 now.

```
% sudo ln -s /usr/include/x86_64-linux-gnu/qt5 /usr/include/qt5
```

### Install dependencies for Ubuntu 14.04 indigo

```
% sudo apt-get install ros-indigo-desktop-full ros-indigo-nmea-msgs ros-indigo-nmea-navsat-driver ros-indigo-sound-play ros-indigo-velodyne-pointcloud
% sudo apt-get install libnlopt-dev freeglut3-dev qtbase5-dev libqt5opengl5-dev libssh2-1-dev libarmadillo-dev
```

### Install Velodyne Driver dependencies 

```
% sudo apt-get install libpcap-dev
% mkdir -p ~/ros_drivers/src
% cd ~/ros_drivers/src 
% catkin_init_workspace
% git clone https://github.com/ros-drivers/velodyne.git
% cd ~/ros_drivers 
% catkin_make
% source devel/setup.bash
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

## Documents

See Autoware/docs. As of Aug 25 2015, we provide only Japanese documents. English documents will be added shortly.

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
