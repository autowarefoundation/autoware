# Note this is the car_demo docker file extracted
# https://github.com/CPFL/Autoware/blob/master/docker/generic/Dockerfile.kinetic
# FROM nvidia/cuda:8.0-devel-ubuntu16.04
################################################################################
# https://github.com/CPFL/Autoware/blob/master/docker/generic/Dockerfile.kinetic
################################################################################
# This is an auto generated Dockerfile for ros:ros-core
# generated from docker_images/create_ros_core_image.Dockerfile.em
FROM ubuntu:xenial
# install packages
RUN apt-get update && apt-get install -y --no-install-recommends \
    dirmngr \
    gnupg2 \
    && rm -rf /var/lib/apt/lists/*

# setup keys
RUN apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-keys 421C365BD9FF1F717815A3895523BAEEB01FA116

# setup sources.list
RUN echo "deb http://packages.ros.org/ros/ubuntu xenial main" > /etc/apt/sources.list.d/ros-latest.list

# install bootstrap tools
RUN apt-get update && apt-get install --no-install-recommends -y \
    python-rosdep \
    python-rosinstall \
    python-vcstools \
    && rm -rf /var/lib/apt/lists/*

# setup environment
ENV LANG C.UTF-8
ENV LC_ALL C.UTF-8

# bootstrap rosdep
RUN rosdep init \
    && rosdep update

# install ros packages
ENV ROS_DISTRO kinetic
RUN apt-get update && apt-get install -y \
    ros-kinetic-ros-core=1.3.1-0* \
    && rm -rf /var/lib/apt/lists/*

# setup entrypoint
COPY ./ros_entrypoint.sh /

#ENTRYPOINT ["/ros_entrypoint.sh"]  # H.F. comment out
#CMD ["bash"]                       # H.F. comment out
################################################################################  END
#https://github.com/CPFL/Autoware/blob/master/docker/generic/Dockerfile.kinetic
################################################################################
#%%%%%%%%
##############################################################
# This is an auto generated Dockerfile for ros:ros-base
# generated from docker_images/create_ros_image.Dockerfile.em
# FROM ros:kinetic-ros-core-xenial  # H.F. comment out

# install ros packages
RUN apt-get update && apt-get install -y \
    ros-kinetic-ros-base=1.3.1-0* \
&& rm -rf /var/lib/apt/lists/*
############################################################  END
#%%%%%%%%
############################################################  START
#https://github.com/osrf/docker_images/blob/5399f380af0a7735405a4b6a07c6c40b867563bd/ros/kinetic/ubuntu/xenial/robot/Dockerfile
##############################################################
# This is an auto generated Dockerfile for ros:robot
# generated from docker_images/create_ros_image.Dockerfile.em
# FROM ros:kinetic-ros-base-xenial  # H.F. comment out

# install ros packages
RUN apt-get update && apt-get install -y \
    ros-kinetic-robot=1.3.1-0* \
&& rm -rf /var/lib/apt/lists/*
############################################################  END
#https://github.com/osrf/docker_images/blob/5399f380af0a7735405a4b6a07c6c40b867563bd/ros/kinetic/ubuntu/xenial/robot/Dockerfile
##############################################################
#%%%%%%%%
############################################################ START
#https://github.com/osrf/docker_images/blob/master/ros/kinetic/ubuntu/xenial/desktop/Dockerfile
##############################################################
# This is an auto generated Dockerfile for ros:desktop
# generated from docker_images/create_ros_image.Dockerfile.em
# FROM ros:kinetic-robot-xenial   # H.F. comment out

# install ros packages
RUN apt-get update && apt-get install -y \
    ros-kinetic-desktop=1.3.1-0* \
&& rm -rf /var/lib/apt/lists/*
############################################################  END
#https://github.com/osrf/docker_images/blob/master/ros/kinetic/ubuntu/xenial/desktop/Dockerfile
##############################################################
#%%%%%%%%
############################################################ START
#https://github.com/osrf/car_demo/blob/master/Dockerfile
##############################################################
# FROM osrf/ros:kinetic-desktop     # H.F. comment out

#LABEL com.nvidia.volumes.needed="nvidia_driver"  # H.F. comment out
#ENV PATH /usr/local/nvidia/bin:${PATH}  # H.F. comment out
#ENV LD_LIBRARY_PATH /usr/local/nvidia/lib:/usr/local/nvidia/lib64:${LD_LIBRARY_PATH}  # H.F. comment out

RUN apt-get update \
 && apt-get install -y \
    wget \
    lsb-release \
    sudo \
    mesa-utils \
 && apt-get clean


# Get gazebo binaries
RUN echo "deb http://packages.osrfoundation.org/gazebo/ubuntu `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list \
 && wget http://packages.osrfoundation.org/gazebo.key -O - | apt-key add - \
 && apt-get update \
 && apt-get install -y \
    gazebo8 \
    ros-kinetic-gazebo8-ros-pkgs \
    ros-kinetic-fake-localization \
    ros-kinetic-joy \
 && apt-get clean


RUN mkdir -p /tmp/workspace/src
COPY ../car_demo/prius_description /tmp/workspace/src/prius_description
COPY ../car_demo/prius_msgs /tmp/workspace/src/prius_msgs
COPY ../car_demo/car_demo /tmp/workspace/src/car_demo
RUN /bin/bash -c 'cd /tmp/workspace \
 && source /opt/ros/kinetic/setup.bash \
 && catkin_make'


CMD ["/bin/bash", "-c", "source /opt/ros/kinetic/setup.bash && source /tmp/workspace/devel/setup.bash && roslaunch car_demo demo.launch"] # H.F. comment out
############################################################  END
#https://github.com/osrf/car_demo/blob/master/Dockerfile
##############################################################
