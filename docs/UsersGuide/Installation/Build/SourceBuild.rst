Source Build
============

Requirements
------------

* ROS Indigo (Ubuntu 14.04) or ROS Kinetic (Ubuntu 16.04)
* OpenCV 2.4.10 or higher
* Qt 5.2.1 or higher
* CUDA (optional)
* FlyCapture2 (optional)
* Armadillo (optional)

ROS
^^^

.. todo::

    Insert link to ROS install

CUDA
^^^^

.. todo::

    Insert link to CUDA install

Install system dependencies for Ubuntu 14.04 Indigo
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. code-block:: shell

    % sudo apt-get install -y  python-catkin-pkg python-rosdep python-wstool ros-$ROS_DISTRO-catkin
    % sudo add-apt-repository ppa:mosquitto-dev/mosquitto-ppa
    % sudo apt-get update
    % sudo apt-get install libmosquitto-dev

.. note::

    NOTE: Please do not install ros-indigo-velodyne-pointcloud package. If it is already installed, please uninstall.

Install system dependencies for Ubuntu 16.04 Kinetic
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. code-block:: shell

    % sudo apt-get update
    % sudo apt-get install -y python-catkin-pkg python-rosdep python-wstool ros-$ROS_DISTRO-catkin libmosquitto-dev

How to Build
------------

1. Clone the repository

.. code-block:: shell

    $ git clone https://github.com/CPFL/Autoware.git --recurse-submodules

.. note::

    If you already have a copy of the repo, run :code:`$ git submodule update --init --recursive`.

2. Initialize the workspace, let rosdep to install the missing dependencies and compile.

.. code-block:: shell

    $ cd Autoware/ros/src
    $ catkin_init_workspace
    $ cd ../
    $ rosdep install -y --from-paths src --ignore-src --rosdistro $ROS_DISTRO
    $ ./catkin_make_release
