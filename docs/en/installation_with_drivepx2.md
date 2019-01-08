# Build and Install Procedure for Nvidia DrivePX2
## Prerequisites
 - Login with user nvidia
 - Internet connectivity

## Caution
 - Install a new DRM related file. Please backup your PX2.

## Procedures
 Launch terminal software and execute the following steps.


### DRM Library Installation
 PX2 has libDRM installed at the time of setup, but NVIDIA customized these libraries and an error occurs for Autoware.  
 So, you need to install common drm libraries.


#### backup
    mkdir -p backup/usr/lib
    sudo cp -a /usr/lib/libdrm* backup/usr/lib
    sudo cp -a /usr/lib/libwayland-* backup/usr/lib

    mkdir -p backup/etc/nvidia
    sudo cp -a /etc/nvidia/nvidia_gl.conf backup/etc/nvidia
    sudo cp -a /etc/nvidia/nvidia_egl.conf backup/etc/nvidia

#### installation
    sudo apt-get install --reinstall -y libdrm2 libdrm-dev libwayland-client0 libwayland-cursor0 libwayland-egl1-mesa libwayland-server0 libwayland-dev
    sudo ldconfig
*Please reboot after installation.


###  ROS Installation
    sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
    sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116
    sudo apt-get update

    sudo apt-get install -y build-essential cmake python-pip
    sudo apt-get install -y checkinstall
    sudo apt-get install -y python-rosdep python-rosinstall-generator python-wstool python-rosinstall build-essential
    sudo apt-get install -y libavutil-ffmpeg54=7:2.8.11-0ubuntu0.16.04.1
    sudo apt-get install -y libswresample-ffmpeg1=7:2.8.11-0ubuntu0.16.04.1
    sudo apt-get install -y libavformat-ffmpeg56=7:2.8.11-0ubuntu0.16.04.1
    sudo apt-get install -y libswscale-ffmpeg3=7:2.8.11-0ubuntu0.16.04.1
    sudo apt-get install -y libssl1.0.0=1.0.2g-1ubuntu4.12
    sudo apt-get install -y libssl-dev=1.0.2g-1ubuntu4.12
    sudo apt-get install -y ros-kinetic-desktop-full
    sudo apt-get install -y ros-kinetic-nmea-msgs ros-kinetic-nmea-navsat-driver ros-kinetic-sound-play ros-kinetic-jsk-visualization ros-kinetic-grid-map ros-kinetic-gps-common
    sudo apt-get install -y ros-kinetic-controller-manager ros-kinetic-ros-control ros-kinetic-ros-controllers ros-kinetic-gazebo-ros-control ros-kinetic-joystick-drivers
    sudo apt-get install -y libnlopt-dev freeglut3-dev qtbase5-dev libqt5opengl5-dev libssh2-1-dev libarmadillo-dev libpcap-dev gksu libgl1-mesa-dev libglew-dev
    sudo apt-get install -y ros-kinetic-camera-info-manager-py ros-kinetic-camera-info-manager

### SSD Installation
If you need to use a SSD detector, please refer SSD readme.

### Autoware Installation
    source /opt/ros/kinetic/setup.bash

    sudo apt-get install -y openssh-server libnlopt-dev freeglut3-dev qtbase5-dev libqt5opengl5-dev libssh2-1-dev libarmadillo-dev libpcap-dev git
    sudo apt-get install -y libnlopt-dev freeglut3-dev qt5-default libqt5opengl5-dev libssh2-1-dev libarmadillo-dev libpcap-dev libglew-dev gksu
    sudo apt-get install -y libxmu-dev python-wxgtk3.0 python-wxgtk3.0-dev
    sudo ln -s /usr/include/aarch64-linux-gnu/qt5 /usr/include/qt5

    cd ~/
    git clone https://github.com/CPFL/Autoware.git
    cd Autoware

    cd ros
    ./catkin_make_release -j1
    (if you need more speed up compilation, you can choose -j2~6, however it may often cause internal erros by GCC.


### Trouble shooting

- If you got following DRM errors
```
    /usr/lib/gcc/aarch64-linux-gnu/5/../../../aarch64-linux-gnu/libGL.so: undefined reference to `drmCloseOnce'
    /usr/lib/gcc/aarch64-linux-gnu/5/../../../aarch64-linux-gnu/libGL.so: undefined reference to `drmMap'
    /usr/lib/gcc/aarch64-linux-gnu/5/../../../aarch64-linux-gnu/libGL.so: undefined reference to `drmUnmap'
    /usr/lib/gcc/aarch64-linux-gnu/5/../../../aarch64-linux-gnu/libGL.so: undefined reference to `drmFreeDevice'
    /usr/lib/gcc/aarch64-linux-gnu/5/../../../aarch64-linux-gnu/libGL.so: undefined reference to `drmGetDevices'
    /usr/lib/gcc/aarch64-linux-gnu/5/../../../aarch64-linux-gnu/libGL.so: undefined reference to `drmGetDevice'
    /usr/lib/gcc/aarch64-linux-gnu/5/../../../aarch64-linux-gnu/libGL.so: undefined reference to `drmOpenOnce'
    /usr/lib/gcc/aarch64-linux-gnu/5/../../../aarch64-linux-gnu/libGL.so: undefined reference to `drmFreeDevices'
```
- please run to following command
```
export LD_LIBRARY_PATH=/usr/lib/aarch64-linux-gnu/
```
