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

## How to setup development environment

```bash
./setup-dev-env.sh
```

This script will install the development environment for Autoware, which cannot be installed by `rosdep`.

> Note: Before installing NVIDIA libraries, confirm and agree with the licenses.

- [CUDA](https://docs.nvidia.com/cuda/eula/index.html)
- [cuDNN](https://docs.nvidia.com/deeplearning/cudnn/sla/index.html)
- [TensorRT](https://docs.nvidia.com/deeplearning/tensorrt/sla/index.html)

## How to setup workspace

```bash
mkdir src
vcs import src < autoware.repos
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
ros2 launch tier4_autoware_launch planning_simulator.launch.xml vehicle_model:=lexus sensor_model:=aip_xx1 map_path:={path_to_your_map_dir}
```
