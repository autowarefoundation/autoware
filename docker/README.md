# Open AD Kit: Containerized Workloads for Autoware

Open AD Kit offers two types of Docker image to let you get started with Autoware quickly: `devel` and `runtime`.

1. The `devel` image enables you to develop Autoware without setting up the local development environment.
2. The `runtime` image contains only runtime executables and enables you to try out Autoware quickly.

!!! info

    Before proceeding, confirm and agree with the [NVIDIA Deep Learning Container license](https://developer.nvidia.com/ngc/nvidia-deep-learning-container-license). By pulling and using the Autoware Open AD Kit images, you accept the terms and conditions of the license.

## Prerequisites

- Docker
- NVIDIA Container Toolkit (preferred)
- NVIDIA CUDA 12 compatible GPU Driver (preferred)

The [setup script](../setup-dev-env.sh) will install all required dependencies with the setup script:

```bash
./setup-dev-env.sh -y docker
```

To install without **NVIDIA GPU** support:

```bash
./setup-dev-env.sh -y --no-nvidia docker
```

!!! info

    GPU acceleration is required for some features such as object detection and traffic light detection/classification. For details of how to enable these features without a GPU, refer to the [Running Autoware without CUDA](../how-to-guides/others/running-autoware-without-cuda.md).

## Usage

### Runtime Setup

You can use `run.sh` to run the Autoware runtime container with the map data:

```bash
./docker/run.sh --map-path path_to_map_data
```

!!! info

    You can use `--no-nvidia` to run without NVIDIA GPU support, and `--headless` to run without display that means no RViz visualization.

For more launch options you can edit the launch command with `--launch-cmd` option:

```bash
./docker/run.sh --map-path path_to_map_data --launch-cmd "ros2 launch autoware_launch autoware.launch.xml map_path:=/autoware_map vehicle_model:=sample_vehicle sensor_model:=sample_sensor_kit"
```

#### Run Autoware simulator

Inside the container, you can run the Autoware simulation by following these tutorials:

[Planning Simulation](../../tutorials/ad-hoc-simulation/planning-simulation.md)

[Rosbag Replay Simulation](../../tutorials/ad-hoc-simulation/rosbag-replay-simulation.md).

### Development Setup

You can use [VS Code Remote Containers](https://marketplace.visualstudio.com/items?itemName=ms-vscode-remote.remote-containers) to develop Autoware in the containerized environment with ease. Or you can use `run.sh` manually to run the Autoware development container with the workspace mounted.

#### Using VS Code Remote Containers for Development

Get the Visual Studio Code's [Remote - Containers](https://marketplace.visualstudio.com/items?itemName=ms-vscode-remote.remote-containers) extension.
And reopen the workspace in the container by selecting `Remote-Containers: Reopen in Container` from the Command Palette (`F1`).

By default devcontainer assumes NVIDIA GPU support, you can change this by deleting these lines within `.devcontainer/devcontainer.json`:

```json
    "hostRequirements": {
      "gpu": true
    },
```

```json
      "--gpus", "all"
```

#### Using `run.sh` for Development

```bash
./docker/run.sh --devel --workspace path_to_workspace
```

!!! info

    You should mount the workspace you are working on with `--workspace path_to_workspace`. For a development environment without NVIDIA GPU support use `--no-nvidia`.

#### Creating a new workspace

If you don't have a workspace yet, you can create a new workspace with following steps:

1. Create the `src` directory and clone repositories into it.

   ```bash
   mkdir src
   vcs import src < autoware.repos
   ```

2. Update dependent ROS packages.

   The dependency of Autoware may change after the Docker image was created.
   In that case, you need to run the following commands to update the dependency.

   ```bash
   sudo apt update
   rosdep update
   rosdep install -y --from-paths src --ignore-src --rosdistro $ROS_DISTRO
   ```

3. Build the workspace.

   ```bash
   colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
   ```

   If there is any build issue, refer to [Troubleshooting](../../support/troubleshooting/index.md#build-issues).

> **To Update the Workspace**
>
> ```bash
> cd autoware
> git pull
> vcs import src < autoware.repos
> vcs pull src
> ```

## Building Docker images from scratch

If you want to build these images locally for development purposes, run the following command:

```bash
cd autoware/
./docker/build.sh
```

To build without CUDA, use the `--no-cuda` option:

```bash
./docker/build.sh --no-cuda
```

To build only development image, use the `--devel-only` option:

```bash
./docker/build.sh --devel-only
```

To specify the platform, use the `--platform` option:

```bash
./docker/build.sh --platform linux/amd64
./docker/build.sh --platform linux/arm64
```

### Using Docker images other than `latest`

There are also images versioned based on the `date` or `release tag`.  
Use them when you need a fixed version of the image.

The list of versions can be found [here](https://github.com/autowarefoundation/autoware/packages).
