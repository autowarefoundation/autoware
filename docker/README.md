# Open AD Kit: Containerized Workloads for the Autoware

Open AD Kit offers two types of Docker image to let you get started with the Autoware quickly: `devel` and `runtime`.

1. The `devel` image contains the development environment and enables you to build and develop Autoware from source. Keep in mind that 'devel' image always include more recent changes than the 'runtime' image to provide the latest development environment.
2. The `runtime` image contains only runtime executables and enables you to try out Autoware quickly. 'runtime' image is more stable than 'devel' image and is recommended for production use.

**Note**: Before proceeding, confirm and agree with the [NVIDIA Deep Learning Container license](https://developer.nvidia.com/ngc/nvidia-deep-learning-container-license). By pulling and using the Autoware Open AD Kit images, you accept the terms and conditions of the license.

## Prerequisites

- Docker
- NVIDIA Container Toolkit (optional)
- NVIDIA CUDA 12 compatible GPU Driver (optional)

The [setup script](../setup-dev-env.sh) will install all required dependencies with the setup script:

```bash
./setup-dev-env.sh --docker
```

To install without NVIDIA GPU support:

```bash
./setup-dev-env.sh --docker --no-nvidia
```

## Usage

### Runtime

You can use `run.sh` to run the Autoware runtime container with the map data:

```bash
./docker/run.sh --map-path path_to_map_data
```

**Note**: You can use `--no-nvidia` to run without NVIDIA GPU support, and `--headless` to run without display that means no RViz visualization.

For more launch options you can edit the launch command `--launch-cmd`:

```bash
./docker/run.sh --map-path path_to_map_data --launch-cmd "ros2 launch autoware_launch autoware.launch.xml map_path:=/autoware_map vehicle_model:=sample_vehicle sensor_model:=sample_sensor_kit"
```

### Using Development Container

```bash
./docker/run.sh --devel
```

**Note**: By default workspace mounted on the container will be current directory, you can change the workspace path by `--workspace path_to_workspace`. For development environments without NVIDIA GPU support use `--no-nvidia`.

#### Using VS Code Remote Containers for Development

Get the Visual Studio Code's [Remote - Containers](https://marketplace.visualstudio.com/items?itemName=ms-vscode-remote.remote-containers) extension.
And reopen the workspace in the container by selecting `Remote-Containers: Reopen in Container` from the Command Palette (`F1`).

By default devcontainer assumes NIVIDA GPU support, you can change this by deleting these lines within `.devcontainer/devcontainer.json`:

```json
    "hostRequirements": {
      "gpu": true
    },
```

```json
      "--gpus", "all"
```

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
