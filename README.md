# Autoware

Open-source software for autonomous driving

## License

* New BSD License
    * See LICENSE

## Requirements

- ROS indigo(Ubuntu 13.10, 14.04) or ROS hydro(Ubuntu 13.04)
- OpenCV 2.4.8 or higher
- Qt 5.2.1 or higher
- CUDA(Optional)
- FlyCapture2(optional)

### Install dependencies for Ubuntu 14.04 indigo

```
% sudo apt-get install ros-indigo-desktop-full ros-indigo-nmea-msgs ros-indigo-sound-play
% sudo apt-get install libnlopt-dev freeglut3-dev qtbase5-dev libqt5opengl5-dev libssh2-1-dev
```

### Install dependencies for Ubuntu 13.10 indigo and Ubuntu 13.04 hydro

```
% sudo apt-get install ros-hydro-desktop-full ros-indigo-nmea-msgs ros-hydro-sound-play
% sudo apt-get install libnlopt-dev freeglut3-dev libssh2-1-dev
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

You cannot build **Autoware/ros** source code with those OpenCV and Qt5 package,
because they are too old. So you have to install newer OpenCV and Qt5.

#### Install OpenCV

You can download source code from [here](http://sourceforge.net/projects/opencvlibrary/).

```
% unzip opencv-2.4.8.zip
% cd opencv-2.4.8
% cmake .
% make
% make install
```

#### Install Qt 5

Document is [here](http://qt-project.org/wiki/Building_Qt_5_from_Git).

First you have to install Qt5 dependencies.

```
% sudo apt-get build-dep qt5-default
% sudo apt-get install build-essential perl python git
% sudo apt-get install "^libxcb.*" libx11-xcb-dev libglu1-mesa-dev libxrender-dev libxi-dev
% sudo apt-get install flex bison gperf libicu-dev libxslt-dev ruby
% sudo apt-get install libssl-dev libxcursor-dev libxcomposite-dev libxdamage-dev libxrandr-dev libfontconfig1-dev
% sudo apt-get install libasound2-dev libgstreamer0.10-dev libgstreamer-plugins-base0.10-dev
```

```
% git clone git://code.qt.io/qt/qt5.git
% cd qt5
% git checkout v5.2.1 # <- Same as Ubuntu 14.04 'qtbase5-dev'
% perl init-repository --no-webkit # <- webkit is very too large
% ./configure -developer-build -opensource -nomake examples -nomake tests # And accept the license 
% make -j # You may take a few hours
% make install
% sudo cp -r qtbase /usr/local/qtbase5
```
[This page](https://qt.gitorious.org/qt/qtbase/commit/9d2edfe5248fce8b16693fad8304f94a1f101bab) could help you when `make` returns error. 

## How to Build

```
$ cd $HOME
$ git clone https://github.com/CPFL/Autoware.git
$ cd ~/Autoware/ros/src
$ catkin_init_workspace
$ cd ../
$ ./catkin_make_release
```
