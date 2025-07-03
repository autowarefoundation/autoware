# hadolint global ignore=DL3006
ARG BASE_IMAGE
FROM $BASE_IMAGE

ENV SHELL /bin/bash

ARG USERNAME=autoware
ARG USER_UID=1000
ARG USER_GID=$USER_UID

RUN groupadd --gid $USER_GID $USERNAME \
  && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
  && apt-get update \
  && apt-get install -y sudo \
  && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
  && chmod 0440 /etc/sudoers.d/$USERNAME


# https://gitlab.corp.dataspeedinc.com/dataspeed/packages
RUN curl "https://gitlab.corp.dataspeedinc.com/api/v4/projects/116/debian_distributions/jammy/key.asc" | gpg --dearmor | sudo tee /usr/share/keyrings/gitlab-packages-archive-keyring.gpg > /dev/null \
    && echo 'deb [ arch=amd64 signed-by=/usr/share/keyrings/gitlab-packages-archive-keyring.gpg ] https://gitlab.corp.dataspeedinc.com/api/v4/projects/116/packages/debian jammy main' | sudo tee /etc/apt/sources.list.d/dataspeed_kinetic_gitlab.list \
    && sudo apt update

# https://bitbucket.org/DataspeedInc/ros_binaries/src/master/
RUN sudo apt install -y curl \
    && sudo curl -sSL https://bitbucket.org/DataspeedInc/ros_binaries/raw/master/dataspeed.key -o /usr/share/keyrings/dataspeed-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/dataspeed-archive-keyring.gpg] http://packages.dataspeedinc.com/ros2/ubuntu $(source /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2-dataspeed-public.list > /dev/null \
    && sudo apt update

RUN sudo sh -c 'echo "yaml http://packages.dataspeedinc.com/ros2/ros-public-'$ROS_DISTRO'.yaml '$ROS_DISTRO'" > /etc/ros/rosdep/sources.list.d/30-dataspeed-public-'$ROS_DISTRO'.list' \
    && sudo rosdep update

RUN sudo apt-get update \
    && sudo apt-get install -y \
        ros-$ROS_DISTRO-dbw-mkz \
        ros-$ROS_DISTRO-dataspeed-boot-usb \
        ros-$ROS_DISTRO-dataspeed-dbw-simulator \
        ros-$ROS_DISTRO-velodyne \
    && sudo apt-get upgrade -y


