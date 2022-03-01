# autoware

This is one of the prototype repositories for Autoware Core/Universe that AWF agreed to create in the [TSC meeting on 2021/11/17](https://discourse.ros.org/t/technical-steering-committee-tsc-meeting-36-2021-11-17-minutes/23168).
Tier IV and AWF members are working in `tier4/proposal` branch for showing the possibility of moving from GitLab to GitHub.

This is the list of the prototype repositories and their roles.

- [autowarefoundation/autoware](https://github.com/autowarefoundation/autoware)
  - This is a meta-repository that contains `.repos` files to construct a workspace.
  - Since it's prospected to be forked by users, we don't put a lot of information here to avoid unnecessary differences.
- [autowarefoundation/autoware_common](https://github.com/autowarefoundation/autoware_common)
  - This is a repository that contains ROS packages referenced in common by many repositories like libraries and utilities.
  - In order to reduce the CI execution time, splitting that kind of packages from a big repository is a good practice.
- [autowarefoundation/autoware.core](https://github.com/autowarefoundation/autoware.core)
  - This is a core repository that contains high-quality and stable ROS packages for Autonomous Driving.
  - Although it's almost empty at this time, it will be implemented based on [Autoware.Auto](https://gitlab.com/autowarefoundation/autoware.auto/AutowareAuto) and [Autoware.Universe](https://github.com/autowarefoundation/autoware.universe) during the next ODD project.
- [autowarefoundation/autoware.universe](https://github.com/autowarefoundation/autoware.universe)
  - This is a core repository that contains experimental but cutting-edge ROS packages for Autonomous Driving.
- [autowarefoundation/autoware_launch](https://github.com/autowarefoundation/autoware_launch)
  - This is a launch configuration repository that contains node configurations and their parameters.
- [autowarefoundation/autoware-github-actions](https://github.com/autowarefoundation/autoware-github-actions)
  - This is a repository for CI that contains [reusable workflows of GitHub Actions](https://docs.github.com/ja/actions/learn-github-actions/reusing-workflows).
  - Since Autoware has a lot of repositories in total, making CI scripts DRY is efficient.
- [autowarefoundation/autoware-documentation](https://github.com/autowarefoundation/autoware-documentation)
  - This is a documentation repository for Autoware users and developers.
  - Since Autoware Core/Universe has multiple repositories, preparing a central documentation repository is more user-friendly than writing distributed documentation in each repository.

If you have any questions or ideas, feel free to start a discussion on [GitHub Discussions in autowarefoundation/autoware](https://github.com/autowarefoundation/autoware/discussions).

---

> Note: Detailed setup documents will be put into [autowarefoundation/autoware-documentation](https://github.com/autowarefoundation/autoware-documentation) soon.

## Installation

### Prerequisite

- [Ubuntu 20.04](https://releases.ubuntu.com/20.04/)
- [Git](https://git-scm.com/)
  - [Registering SSH keys to GitHub](https://github.com/settings/keys) is preferable.

```bash
sudo apt-get -y update
sudo apt-get -y install git
```

### How to set up a development environment

There are two ways of installation. Choose one depending on your preference.

1. Docker installation

   [Docker](https://www.docker.com/) can ensure that all developers in a project have a common, consistent development environment.
   It is recommended for beginners, light users, people who do not use Ubuntu.

2. Source installation

   Source installation is for the cases where more granular control of the installation environment is needed.
   It is recommended for skilled users or people who want to customize the environment.

   Note that some problems may occur depending on your local environment.

#### Docker installation

1. Clone `autowarefoundation/autoware` and move to the directory.

   ```bash
   git clone https://github.com/autowarefoundation/autoware.git
   cd autoware
   ```

2. Install the dependencies.

   ```bash
   ./setup-dev-env.sh docker
   ```

See [docker/README.md](docker/README.md) for the usage.

#### Source installation

1. Clone `autowarefoundation/autoware` and move to the directory.

   ```bash
   git clone https://github.com/autowarefoundation/autoware.git
   cd autoware
   ```

2. Install the dependencies.

   > Note: Before installing NVIDIA libraries, confirm and agree with the licenses.

   - [CUDA](https://docs.nvidia.com/cuda/eula/index.html)
   - [cuDNN](https://docs.nvidia.com/deeplearning/cudnn/sla/index.html)
   - [TensorRT](https://docs.nvidia.com/deeplearning/tensorrt/sla/index.html)

   ```bash
   ./setup-dev-env.sh
   ```

### How to set up a workspace

Suppose that you've installed a development environment.

1. Create the `src` directory and clone repositories into it.

   ```bash
   mkdir src
   vcs import src < autoware.repos
   ```

2. Install dependent ROS packages.

   ```bash
   source /opt/ros/galactic/setup.bash
   rosdep install --from-paths src --ignore-src --rosdistro $ROS_DISTRO
   ```

3. Build the workspace.

   ```bash
   colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
   ```

## Usage

Suppose that you've set up a workspace.

### How to run a closed-loop simulation

1. Download a sample map [here](https://drive.google.com/file/d/197kgRfSomZzaSbRrjWTx614le2qN-oxx/view) and unzip it.

   ```bash
   cd ~/Downloads
   unzip planning_simulator_sample_map.zip -d sample_map
   ```

2. Launch a simulation.

   ```bash
   source install/setup.bash
   ros2 launch autoware_launch planning_simulator.launch.xml vehicle_model:=sample_vehicle sensor_model:=sample_sensor_kit map_path:=$HOME/Downloads/sample_map
   ```

> Note: More tutorials will be written [here](https://autowarefoundation.github.io/autoware-documentation/tier4-proposal/) soon.
