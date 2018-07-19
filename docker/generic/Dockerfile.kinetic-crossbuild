ARG DOCKER_ARCH
ARG SYSROOT_ARCH
FROM multiarch/alpine:${SYSROOT_ARCH}-latest-stable AS bootstrap

FROM ${DOCKER_ARCH}/ubuntu:16.04 AS sysroot
ARG SYSROOT_ARCH

COPY --from=bootstrap /usr/bin/qemu-${SYSROOT_ARCH}-static /usr/bin/qemu-${SYSROOT_ARCH}-static

RUN echo "deb http://packages.ros.org/ros/ubuntu xenial main" | tee /etc/apt/sources.list.d/ros-latest.list
RUN apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116

RUN apt-get update && apt-get install -y --no-install-recommends \
    curl \
    lsb-release \
    python-rosdep \
    sudo

# Autoware dependencies
RUN apt-get update && apt-get install -y --no-install-recommends \
    freeglut3-dev \
    libarmadillo-dev \
    libcurl4-openssl-dev \
    libeigen3-dev \
    libgflags-dev \
    libgl1-mesa-dev \
    libglew-dev \
    libglu1-mesa-dev \
    libgoogle-glog-dev \
    libgtest-dev \
    libnlopt-dev \
    libopencv-dev \
    libpcap0.8-dev \
    libpcl-dev \
    libpcl1.7 \
    libqt5core5a \
    libqt5gui5 \
    libqt5opengl5 \
    libqt5opengl5-dev \
    libqt5widgets5 \
    libssh2-1 \
    libtinyxml-dev \
    libx11-dev \
    libxi-dev \
    libxml2-dev \
    libxmu-dev \
    libyaml-cpp-dev \
    python-flask \
    python-serial \
    qtbase5-dev \
    ros-kinetic-angles \
    ros-kinetic-camera-info-manager \
    ros-kinetic-catkin \
    ros-kinetic-cmake-modules \
    ros-kinetic-cv-bridge \
    ros-kinetic-diagnostic-msgs \
    ros-kinetic-diagnostic-updater \
    ros-kinetic-dynamic-reconfigure \
    ros-kinetic-geometry-msgs \
    ros-kinetic-gps-common \
    ros-kinetic-grid-map-cv \
    ros-kinetic-grid-map-msgs \
    ros-kinetic-grid-map-ros \
    ros-kinetic-image-geometry \
    ros-kinetic-image-transport \
    ros-kinetic-imu-filter-madgwick \
    ros-kinetic-imu-tools \
    ros-kinetic-jsk-recognition-msgs \
    ros-kinetic-jsk-rviz-plugins \
    ros-kinetic-message-filters \
    ros-kinetic-message-generation \
    ros-kinetic-message-runtime \
    ros-kinetic-nav-msgs \
    ros-kinetic-nlopt \
    ros-kinetic-nmea-msgs \
    ros-kinetic-nodelet \
    ros-kinetic-pcl-conversions \
    ros-kinetic-pcl-msgs \
    ros-kinetic-pcl-ros \
    ros-kinetic-pluginlib \
    ros-kinetic-rosconsole \
    ros-kinetic-roscpp \
    ros-kinetic-roslaunch \
    ros-kinetic-roslib \
    ros-kinetic-roslint \
    ros-kinetic-rospy \
    ros-kinetic-rostest \
    ros-kinetic-rosunit \
    ros-kinetic-rqt-plot \
    ros-kinetic-rviz \
    ros-kinetic-sensor-msgs \
    ros-kinetic-shape-msgs \
    ros-kinetic-sound-play \
    ros-kinetic-std-msgs \
    ros-kinetic-std-srvs \
    ros-kinetic-stereo-msgs \
    ros-kinetic-tf \
    ros-kinetic-tf2 \
    ros-kinetic-tf2-ros \
    ros-kinetic-visualization-msgs \
    ros-kinetic-xacro

RUN find / -depth -xdev -type l -lname '/*' -exec sh -c 'linkpath="$(readlink {})" && rm -f {} && ln -frsv "$linkpath" "{}"' \;

FROM ubuntu:16.04 AS builder
ARG SYSROOT_ARCH
COPY --from=sysroot /lib /sysroot/${SYSROOT_ARCH}/lib
COPY --from=sysroot /usr/include /sysroot/${SYSROOT_ARCH}/usr/include
COPY --from=sysroot /usr/lib /sysroot/${SYSROOT_ARCH}/usr/lib
COPY --from=sysroot /usr/share/pkgconfig /sysroot/${SYSROOT_ARCH}/usr/share/pkgconfig
COPY --from=sysroot /opt /sysroot/${SYSROOT_ARCH}/opt
COPY --from=sysroot /etc/alternatives /sysroot/${SYSROOT_ARCH}/etc/alternatives
RUN echo "deb http://packages.ros.org/ros/ubuntu xenial main" | tee /etc/apt/sources.list.d/ros-latest.list
RUN apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116
RUN apt-get update && apt-get install -y --no-install-recommends \
    crossbuild-essential-arm64 \
    pkg-config \
    python-rospkg \
    qt5-qmake \
    qtbase5-dev-tools \
    ros-kinetic-catkin
RUN find /sysroot/${SYSROOT_ARCH}/opt/ros/kinetic/share/ -name "*.cmake" -type f -exec sed -i -e "s#/opt/ros/kinetic/include#/sysroot/${SYSROOT_ARCH}/opt/ros/kinetic/include#g" {} \;
RUN find /sysroot/${SYSROOT_ARCH}/opt/ros/kinetic/share/ -name "*.cmake" -type f -exec sed -i -e "s#/opt/ros/kinetic/lib#/sysroot/${SYSROOT_ARCH}/opt/ros/kinetic/lib#g" {} \;
RUN find /sysroot/${SYSROOT_ARCH}/opt/ros/kinetic/share/ -name "*.cmake" -type f -exec sed -i -e "s#/usr/include#/sysroot/${SYSROOT_ARCH}/usr/include#g" {} \;
RUN find /sysroot/${SYSROOT_ARCH}/opt/ros/kinetic/share/ -name "*.cmake" -type f -exec sed -i -e "s#/usr/lib#/sysroot/${SYSROOT_ARCH}/usr/lib#g" {} \;
RUN find /sysroot/${SYSROOT_ARCH}/usr/lib/cmake/ -name "*.cmake" -type f -exec sed -i -e "s#/usr/lib/${SYSROOT_ARCH}-linux-gnu#/sysroot/${SYSROOT_ARCH}/usr/lib/${SYSROOT_ARCH}-linux-gnu#g" {} \;
RUN find /sysroot/${SYSROOT_ARCH}/usr/lib/cmake/ -name "*.cmake" -type f -exec sed -i -e "s#/usr/lib/openmpi#/sysroot/${SYSROOT_ARCH}/usr/lib/openmpi#g" {} \;
RUN find /sysroot/${SYSROOT_ARCH}/usr/lib/cmake/ -name "*.cmake" -type f -exec sed -i -e "s#/usr/include#/sysroot/${SYSROOT_ARCH}/usr/include#g" {} \;
RUN find /sysroot/${SYSROOT_ARCH}/opt/ros/kinetic/lib/pkgconfig/ -name "*.pc" -type f -exec sed -i -e "s#-I/opt/ros/kinetic/usr/include#-I/sysroot/${SYSROOT_ARCH}/opt/ros/kinetic/usr/include#g" {} \;
RUN find /sysroot/${SYSROOT_ARCH}/opt/ros/kinetic/lib/pkgconfig/ -name "*.pc" -type f -exec sed -i -e "s#-I/usr/include#-I/sysroot/${SYSROOT_ARCH}/usr/include#g" {} \;
RUN find /sysroot/${SYSROOT_ARCH}/ -name "*.pc" -type f -exec sed -i -e "s#prefix=/#prefix=/sysroot/${SYSROOT_ARCH}/#g" {} \;
RUN sed -i -e "s#/usr#/sysroot/${SYSROOT_ARCH}/usr#g" /sysroot/${SYSROOT_ARCH}/usr/lib/${SYSROOT_ARCH}-linux-gnu/cmake/pcl/PCLConfig.cmake
RUN sed -i -e "s#set(imported_location \"\${_qt5Widgets_install_prefix}/lib/${SYSROOT_ARCH}-linux-gnu/qt5/bin/uic\")#set(imported_location \"/usr/lib/x86_64-linux-gnu/qt5/bin/uic\")#g" \
    /sysroot/${SYSROOT_ARCH}/usr/lib/${SYSROOT_ARCH}-linux-gnu/cmake/Qt5Widgets/Qt5WidgetsConfigExtras.cmake
RUN sed -i -e "s#set(imported_location \"\${_qt5Core_install_prefix}/lib/${SYSROOT_ARCH}-linux-gnu/qt5/bin/#set(imported_location \"/usr/lib/x86_64-linux-gnu/qt5/bin/#g" \
    /sysroot/${SYSROOT_ARCH}/usr/lib/${SYSROOT_ARCH}-linux-gnu/cmake/Qt5Core/Qt5CoreConfigExtras.cmake
RUN sed -i -e "s#define ARMA_SUPERLU_INCLUDE_DIR /usr/include/superlu/#define ARMA_SUPERLU_INCLUDE_DIR /sysroot/${SYSROOT_ARCH}/usr/include/superlu/#g" \
    /sysroot/${SYSROOT_ARCH}/usr/include/armadillo_bits/config.hpp
ENV ROS_DISTRO kinetic
ENV CC /usr/bin/${SYSROOT_ARCH}-linux-gnu-gcc
ENV CXX /usr/bin/${SYSROOT_ARCH}-linux-gnu-g++
ENV AR /usr/bin/${SYSROOT_ARCH}-linux-gnu-ar
ENV CPP /usr/bin/${SYSROOT_ARCH}-linux-gnu-cpp
CMD . /opt/ros/kinetic/setup.sh && /bin/bash
