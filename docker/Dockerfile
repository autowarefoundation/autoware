FROM osrf/ros:kinetic-desktop
# FROM nvidia/cuda:8.0-devel-ubuntu16.04

# for testing; to save time consider having travis build this: https://docs.travis-ci.com/user/docker/

# to get ride of warning: https://github.com/phusion/baseimage-docker/issues/58
RUN echo 'debconf debconf/frontend select Noninteractive' | debconf-set-selections

MAINTAINER Yuki Iida <aiueo.0409@gmail.com>

LABEL com.nvidia.volumes.needed="nvidia_driver"
ENV PATH /usr/local/nvidia/bin:${PATH}
ENV LD_LIBRARY_PATH /usr/local/nvidia/lib:/usr/local/nvidia/lib64:${LD_LIBRARY_PATH}

RUN apt-get update \
 && apt-get install -y \
        software-properties-common \
        wget \
        lsb-release \
        sudo \
        mesa-utils \
        curl \
        git \
        cmake \
        cmake-curses-gui \
        apt-utils \
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

 # Install other ROS packages
 RUN apt-get install -y \
     ros-kinetic-nmea-msgs \
     ros-kinetic-nmea-navsat-driver ros-kinetic-sound-play \
     ros-kinetic-jsk-visualization \
     ros-kinetic-perception-pcl \
     ros-kinetic-openni-launch \
     ros-kinetic-velocity-controllers  ros-kinetic-grid-map \
     ros-kinetic-gps-common \
     ros-kinetic-ros-control ros-kinetic-ros-controllers \
  && apt-get clean

  # currently these packages are causing errors:
          #ros-kinetic-turtlebot-simulator \
          #ros-jade-sicktoolbox ros-jade-sicktoolbox-wrapper \
          #ros-kinetic-gazebo-ros-control \

# Install other packages
RUN apt-get install -y \
    libnlopt-dev freeglut3-dev qtbase5-dev \
    libqt5opengl5-dev libssh2-1-dev libarmadillo-dev libpcap-dev gksu \
    libglew-dev \
    libopenni2-dev \
&& apt-get clean
# added:  libopenni2-dev to fix: ** WARNING ** visualization features related to openni2 will be disabled

# Update ros dependencies
# NOTE: OK to ignore the warning
# Warning: running 'rosdep update' as root is not recommended.
#  You should run 'sudo rosdep fix-permissions' and invoke 'rosdep update' again without sudo.
# as per; https://answers.ros.org/question/241112/how-to-run-rosdep-init-and-update-in-dockerfile/
# RUN rosdep init \  # TODO: make sure this has already been done, if it has and this command is run, an error will be produced.
#        && rosdep update \
RUN rosdep update \
        && echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc

# Develop
RUN apt-get install -y \
        libboost-all-dev \
        libflann-dev \
        libgsl0-dev \
        libgoogle-perftools-dev \
        libeigen3-dev

# Install some basic GUI and sound libs
RUN apt-get install -y \
                xz-utils file locales dbus-x11 pulseaudio dmz-cursor-theme \
                fonts-dejavu fonts-liberation hicolor-icon-theme \
                libcanberra-gtk3-0 libcanberra-gtk-module libcanberra-gtk3-module \
                libasound2 libgtk2.0-0 libdbus-glib-1-2 libxt6 libexif12 \
                libgl1-mesa-glx libgl1-mesa-dri \
        && update-locale LANG=C.UTF-8 LC_MESSAGES=POSIX

# Install some basic GUI tools
RUN apt-get install -y \
        cmake-qt-gui \
        gnome-terminal

# Install OpenCV  #TODO: fix depreciation warnings, probably due to the fact that the version of NVIDIA that is being used is not:
 # FROM nvidia/cuda:8.0-devel-ubuntu14.04
 # To fix, might try and have multiple build stages as: https://docs.docker.com/engine/userguide/eng-image/multistage-build/#name-your-build-stages
        # where on of the stages is: nvidia/cuda:8.0-devel-ubuntu14.04
RUN apt-get update && apt-get -y install libopencv-dev build-essential cmake git libgtk2.0-dev pkg-config python-dev python-numpy libdc1394-22 libdc1394-22-dev libjpeg-dev libpng12-0 libjasper-dev libavcodec-dev libavformat-dev libswscale-dev libgstreamer0.10-dev libgstreamer-plugins-base0.10-dev libv4l-dev libtbb-dev libqt4-dev libmp3lame-dev libopencore-amrnb-dev libopencore-amrwb-dev libtheora-dev libvorbis-dev libxvidcore-dev x264 v4l-utils unzip

RUN wget https://github.com/opencv/opencv/archive/2.4.13.zip && \
    unzip 2.4.13.zip && \
    rm 2.4.13.zip && \
    cd opencv-2.4.13/ && \
    mkdir build && \
    cd build/ && \
    cmake -D WITH_TBB=ON -D BUILD_NEW_PYTHON_SUPPORT=ON -D WITH_V4L=ON -D CUDA_GENERATION=Auto -D WITH_QT=ON -D WITH_OPENGL=ON -D WITH_VTK=ON .. && \
    make -j8 && \
    make install
ENV PKG_CONFIG_PATH $PKG_CONFIG_PATH:/usr/local/lib/pkgconfig

# Add basic user
# Replace 1000 with your user/group id
ENV USERNAME autoware
ENV PULSE_SERVER /run/pulse/native
RUN useradd -m $USERNAME && \
        echo "$USERNAME:$USERNAME" | chpasswd && \
        usermod --shell /bin/bash $USERNAME && \
        usermod -aG sudo $USERNAME && \
        echo "$USERNAME ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers.d/$USERNAME && \
        chmod 0440 /etc/sudoers.d/$USERNAME && \
        usermod  --uid 1000 $USERNAME && \
        groupmod --gid 1000 $USERNAME

# Setup .bashrc for ROS
# Fix for qt and X server errors
# echo "export QT_X11_NO_MITSHM=1" >> /home/$USERNAME/.bashrc && \
# cd to home on login
# echo "cd" >> /home/$USERNAME/.bashrc
RUN echo "source /opt/ros/kinetic/setup.bash" >> /home/$USERNAME/.bashrc && \
        echo "export QT_X11_NO_MITSHM=1" >> /home/$USERNAME/.bashrc && \
        echo "cd" >> /home/$USERNAME/.bashrc

# Change user
USER autoware

# Install Autoware
RUN git clone https://github.com/CPFL/Autoware.git /home/$USERNAME/Autoware
RUN /bin/bash -c 'source /opt/ros/kinetic/setup.bash \
 && cd /home/$USERNAME/Autoware/ros/src \
 && catkin_init_workspace \
 && cd ../; ./catkin_make_release'

RUN echo "source /home/$USERNAME/Autoware/ros/devel/setup.bash" >> /home/$USERNAME/.bashrc
#RUN echo "export NO_AT_BRIDGE=1" >> /home/$USERNAME/.bashrc #  https://bugs.launchpad.net/ubuntu/+source/at-spi2-core/+bug/1193236

# Change Terminal Color
RUN gconftool-2 --set "/apps/gnome-terminal/profiles/Default/use_theme_background" --type bool false
RUN gconftool-2 --set "/apps/gnome-terminal/profiles/Default/use_theme_colors" --type bool false
RUN gconftool-2 --set "/apps/gnome-terminal/profiles/Default/background_color" --type string "#FFFFFF"

# Default CMD
CMD ["gnome-terminal" ]
