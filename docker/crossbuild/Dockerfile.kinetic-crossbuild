ARG AUTOWARE_DOCKER_ARCH
ARG AUTOWARE_TARGET_ARCH
FROM multiarch/alpine:${AUTOWARE_TARGET_ARCH}-latest-stable AS bootstrap

FROM ${AUTOWARE_DOCKER_ARCH}/ubuntu:16.04 AS sysroot
ARG AUTOWARE_TARGET_ARCH

COPY --from=bootstrap /usr/bin/qemu-${AUTOWARE_TARGET_ARCH}-static /usr/bin/qemu-${AUTOWARE_TARGET_ARCH}-static

RUN echo "deb http://packages.ros.org/ros/ubuntu xenial main" | tee /etc/apt/sources.list.d/ros-latest.list
RUN apt-key adv --keyserver hkp://p80.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116

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
ARG AUTOWARE_TARGET_ARCH
ARG AUTOWARE_TARGET_PLATFORM
ENV AUTOWARE_SYSROOT /sysroot/${AUTOWARE_TARGET_PLATFORM}
COPY --from=sysroot /lib ${AUTOWARE_SYSROOT}/lib
COPY --from=sysroot /usr/include ${AUTOWARE_SYSROOT}/usr/include
COPY --from=sysroot /usr/lib ${AUTOWARE_SYSROOT}/usr/lib
COPY --from=sysroot /usr/share/pkgconfig ${AUTOWARE_SYSROOT}/usr/share/pkgconfig
COPY --from=sysroot /opt ${AUTOWARE_SYSROOT}/opt
COPY --from=sysroot /etc/alternatives ${AUTOWARE_SYSROOT}/etc/alternatives
RUN echo "deb http://packages.ros.org/ros/ubuntu xenial main" | tee /etc/apt/sources.list.d/ros-latest.list
RUN apt-key adv --keyserver hkp://p80.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116
RUN apt-get update && apt-get install -y --no-install-recommends \
    crossbuild-essential-arm64 \
    pkg-config \
    python-rospkg \
    qt5-qmake \
    qtbase5-dev-tools \
    ros-kinetic-catkin
RUN find ${AUTOWARE_SYSROOT}/opt/ros/kinetic/share/ -name "*.cmake" -type f -exec sed -i -e "s#/opt/ros/kinetic/include#${AUTOWARE_SYSROOT}/opt/ros/kinetic/include#g" {} \;
RUN find ${AUTOWARE_SYSROOT}/opt/ros/kinetic/share/ -name "*.cmake" -type f -exec sed -i -e "s#/opt/ros/kinetic/lib#${AUTOWARE_SYSROOT}/opt/ros/kinetic/lib#g" {} \;
RUN find ${AUTOWARE_SYSROOT}/opt/ros/kinetic/share/ -name "*.cmake" -type f -exec sed -i -e "s#/usr/include#${AUTOWARE_SYSROOT}/usr/include#g" {} \;
RUN find ${AUTOWARE_SYSROOT}/opt/ros/kinetic/share/ -name "*.cmake" -type f -exec sed -i -e "s#/usr/lib#${AUTOWARE_SYSROOT}/usr/lib#g" {} \;
RUN find ${AUTOWARE_SYSROOT}/usr/lib/cmake/ -name "*.cmake" -type f -exec sed -i -e "s#/usr/lib/${AUTOWARE_TARGET_ARCH}-linux-gnu#${AUTOWARE_SYSROOT}/usr/lib/${AUTOWARE_TARGET_ARCH}-linux-gnu#g" {} \;
RUN find ${AUTOWARE_SYSROOT}/usr/lib/cmake/ -name "*.cmake" -type f -exec sed -i -e "s#/usr/lib/openmpi#${AUTOWARE_SYSROOT}/usr/lib/openmpi#g" {} \;
RUN find ${AUTOWARE_SYSROOT}/usr/lib/cmake/ -name "*.cmake" -type f -exec sed -i -e "s#/usr/include#${AUTOWARE_SYSROOT}/usr/include#g" {} \;
RUN find ${AUTOWARE_SYSROOT}/opt/ros/kinetic/lib/pkgconfig/ -name "*.pc" -type f -exec sed -i -e "s#-I/opt/ros/kinetic/usr/include#-I${AUTOWARE_SYSROOT}/opt/ros/kinetic/usr/include#g" {} \;
RUN find ${AUTOWARE_SYSROOT}/opt/ros/kinetic/lib/pkgconfig/ -name "*.pc" -type f -exec sed -i -e "s#-I/usr/include#-I${AUTOWARE_SYSROOT}/usr/include#g" {} \;
RUN find ${AUTOWARE_SYSROOT}/ -name "*.pc" -type f -exec sed -i -e "s#prefix=/#prefix=${AUTOWARE_SYSROOT}/#g" {} \;
RUN sed -i -e "s#/usr#${AUTOWARE_SYSROOT}/usr#g" ${AUTOWARE_SYSROOT}/usr/lib/${AUTOWARE_TARGET_ARCH}-linux-gnu/cmake/pcl/PCLConfig.cmake
RUN sed -i -e "s#set(imported_location \"\${_qt5Widgets_install_prefix}/lib/${AUTOWARE_TARGET_ARCH}-linux-gnu/qt5/bin/uic\")#set(imported_location \"/usr/lib/x86_64-linux-gnu/qt5/bin/uic\")#g" \
    ${AUTOWARE_SYSROOT}/usr/lib/${AUTOWARE_TARGET_ARCH}-linux-gnu/cmake/Qt5Widgets/Qt5WidgetsConfigExtras.cmake
RUN sed -i -e "s#set(imported_location \"\${_qt5Core_install_prefix}/lib/${AUTOWARE_TARGET_ARCH}-linux-gnu/qt5/bin/#set(imported_location \"/usr/lib/x86_64-linux-gnu/qt5/bin/#g" \
    ${AUTOWARE_SYSROOT}/usr/lib/${AUTOWARE_TARGET_ARCH}-linux-gnu/cmake/Qt5Core/Qt5CoreConfigExtras.cmake
RUN sed -i -e "s#define ARMA_SUPERLU_INCLUDE_DIR /usr/include/superlu/#define ARMA_SUPERLU_INCLUDE_DIR ${AUTOWARE_SYSROOT}/usr/include/superlu/#g" \
    ${AUTOWARE_SYSROOT}/usr/include/armadillo_bits/config.hpp
ENV ROS_DISTRO kinetic
ENV CC /usr/bin/${AUTOWARE_TARGET_ARCH}-linux-gnu-gcc
ENV CXX /usr/bin/${AUTOWARE_TARGET_ARCH}-linux-gnu-g++
ENV AR /usr/bin/${AUTOWARE_TARGET_ARCH}-linux-gnu-ar
ENV CPP /usr/bin/${AUTOWARE_TARGET_ARCH}-linux-gnu-cpp
CMD . /opt/ros/kinetic/setup.sh && /bin/bash
