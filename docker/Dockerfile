FROM ubuntu:14.04
MAINTAINER Yuki Iida <aiueo.0409@gmail.com>

# Intall ROS
RUN apt-get update && apt-get install -y \
        software-properties-common \
        wget

RUN wget http://packages.ros.org/ros.key -O - | apt-key add -
RUN echo "deb http://packages.ros.org/ros/ubuntu trusty main" > /etc/apt/sources.list.d/ros-latest.list
RUN apt-get update && apt-get install -y \
        ros-indigo-desktop-full ros-indigo-nmea-msgs ros-indigo-nmea-navsat-driver ros-indigo-sound-play ros-indigo-jsk-visualization && apt-get install -y \
        libnlopt-dev freeglut3-dev qtbase5-dev libqt5opengl5-dev libssh2-1-dev libarmadillo-dev libpcap-dev gksu

RUN rosdep init \
        && rosdep update \
        && echo "source /opt/ros/indigo/setup.bash" >> ~/.bashrc

# Intall some basic CLI tools
RUN apt-get install -y \
        curl \
        screen \
        byobu \
        fish \
        git \
        nano \
        glances

# Develop
RUN apt-get install -y \
        cmake \
        cmake-curses-gui

RUN apt-get install -y \
        libboost-all-dev \
        libflann-dev \
        libgsl0-dev \
        libgoogle-perftools-dev

RUN apt-get install -y \
        libeigen3-dev \
        ros-indigo-perception-pcl \
        ros-indigo-openni-launch \
        ros-indigo-turtlebot-simulator

# Intall some basic GUI and sound libs
RUN apt-get install -y \
                xz-utils file locales dbus-x11 pulseaudio dmz-cursor-theme \
                fonts-dejavu fonts-liberation hicolor-icon-theme \
                libcanberra-gtk3-0 libcanberra-gtk-module libcanberra-gtk3-module \
                libasound2 libgtk2.0-0 libdbus-glib-1-2 libxt6 libexif12 \
                libgl1-mesa-glx libgl1-mesa-dri \
        && update-locale LANG=C.UTF-8 LC_MESSAGES=POSIX

# Intall some basic GUI tools
RUN apt-get install -y \
        terminator \
        cmake-qt-gui \
        gedit

# Install NVIDIA Driver
RUN apt-get update && sudo apt-get install -y \
        x-window-system \
        binutils \
        mesa-utils \
        module-init-tools

ADD nvidia-driver.run /tmp/nvidia-driver.run
RUN sh /tmp/nvidia-driver.run -a -N --ui=none --no-kernel-module \
        && sudo rm /tmp/nvidia-driver.run

# Install CUDA
RUN wget http://developer.download.nvidia.com/compute/cuda/repos/ubuntu1404/x86_64/cuda-repo-ubuntu1404_6.5-14_amd64.deb
RUN dpkg -i cuda-repo-ubuntu1404_6.5-14_amd64.deb
RUN rm -f cuda-repo-ubuntu1404_6.5-14_amd64.deb
RUN apt-get update
RUN apt-get -y install cuda || echo ignore error

ENV CUDA_HOME $CUDA_HOME:/usr/local/cuda
ENV LD_LIBRARY_PATH $LD_LIBRARY_PATH:$CUDA_HOME/lib64
ENV PATH $PATH:$CUDA_HOME/bin

# Install OpenCV
RUN apt-get update && apt-get -y install libopencv-dev build-essential cmake git libgtk2.0-dev pkg-config python-dev python-numpy libdc1394-22 libdc1394-22-dev libjpeg-dev libpng12-0 libjasper-dev libavcodec-dev libavformat-dev libswscale-dev libgstreamer0.10-dev libgstreamer-plugins-base0.10-dev libv4l-dev libtbb-dev libqt4-dev libmp3lame-dev libopencore-amrnb-dev libopencore-amrwb-dev libtheora-dev libvorbis-dev libxvidcore-dev x264 v4l-utils unzip

RUN wget https://github.com/opencv/opencv/archive/2.4.11.zip && \
    unzip 2.4.11.zip && \
    cd opencv-2.4.11/ && \
    mkdir build && \
    cd build/ && \
    cmake -D WITH_TBB=ON -D BUILD_NEW_PYTHON_SUPPORT=ON -D WITH_V4L=ON -D INSTALL_C_EXAMPLES=ON -D INSTALL_PYTHON_EXAMPLES=ON -D BUILD_EXAMPLES=ON -D CUDA_GENERATION=Fermi -D WITH_QT=ON -D WITH_OPENGL=ON -D WITH_VTK=ON .. && \
    make -j8 && \
    make install 
ENV PKG_CONFIG_PATH $PKG_CONFIG_PATH:/usr/local/lib/pkgconfig

# Add basic user
ENV USERNAME autoware
ENV PULSE_SERVER /run/pulse/native
RUN useradd -m $USERNAME && \
        echo "$USERNAME:$USERNAME" | chpasswd && \
        usermod --shell /bin/bash $USERNAME && \
        usermod -aG sudo $USERNAME && \
        echo "$USERNAME ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers.d/$USERNAME && \
        chmod 0440 /etc/sudoers.d/$USERNAME && \
        # Replace 1000 with your user/group id
        usermod  --uid 1000 $USERNAME && \
        groupmod --gid 1000 $USERNAME

# Setup .bashrc for ROS
RUN echo "source /opt/ros/indigo/setup.bash" >> /home/$USERNAME/.bashrc && \
        #Fix for qt and X server errors
        echo "export QT_X11_NO_MITSHM=1" >> /home/$USERNAME/.bashrc && \
        # cd to home on login
        echo "cd" >> /home/$USERNAME/.bashrc 

# Change user
USER autoware


# Install Autoware
# Autoware Install.
RUN git clone https://github.com/CPFL/Autoware.git /home/$USERNAME/Autoware
RUN /bin/bash -c 'source /opt/ros/indigo/setup.bash; cd /home/$USERNAME/Autoware/ros/src; catkin_init_workspace; cd ../; ./catkin_make_release'
RUN echo "source /home/$USERNAME/Autoware/ros/devel/setup.bash" >> /home/$USERNAME/.bashrc

CMD ["terminator"]
