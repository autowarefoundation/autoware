# Run Autoware in Docker

## Image Graph

```mermaid
graph TD
    base(["base"])
    base --> core-dependencies(["core-dependencies"])
    core-dependencies --> core-devel(["core-devel"])
    core-devel --> universe-dependencies(["universe-dependencies"])
    universe-dependencies --> universe-dependencies-cuda(["universe-dependencies-cuda"])
    universe-dependencies --> universe-devel(["universe-devel"])
    universe-dependencies-cuda --> universe-devel-cuda(["universe-devel-cuda"])
    base --> core(["core"])
    core-devel -- " COPY /opt/autoware " --> core
    core --> universe-runtime-dependencies(["universe-runtime-dependencies"])
    universe-runtime-dependencies --> universe(["universe"])
    universe-runtime-dependencies --> universe-cuda(["universe-cuda"])
    universe-devel -- " COPY /opt/autoware " --> universe
    universe-devel-cuda -- " COPY /opt/autoware " --> universe-cuda
    classDef base fill: #e8e8e8, color: #333
    classDef devel fill: #bbdefb, color: #333
    classDef runtime fill: #c8e6c9, color: #333
    classDef cuda fill: #e1bee7, color: #333
    class base base
    class core-dependencies,core-devel,universe-dependencies,universe-devel devel
    class core,universe-runtime-dependencies,universe runtime
    class universe-dependencies-cuda,universe-devel-cuda,universe-cuda cuda
```

## Images

| Image                           | Description                                                                          | Use case                                                   |
| ------------------------------- | ------------------------------------------------------------------------------------ | ---------------------------------------------------------- |
| `base`                          | ROS base, sudo, pipx, ansible, RMW, user `aw`                                        | Foundation for all other images                            |
| `core-dependencies`             | Build deps + compiled core packages (except autoware_core and autoware_rviz_plugins) | CI for autoware_core                                       |
| `core-devel`                    | Adds autoware_core build on top of core-dependencies                                 | Development and CI for packages depending on autoware_core |
| `core`                          | Runtime-only: rosdep exec deps + compiled core from core-devel                       | Lightweight core runtime                                   |
| `universe-dependencies`         | Ansible universe roles + rosdep build deps for all of autoware                       | CI for autoware_universe                                   |
| `universe-dependencies-cuda`    | Adds CUDA, TensorRT, spconv dev libs                                                 | CI for CUDA-dependent packages                             |
| `universe-devel`                | Builds all universe sources (no CUDA)                                                | Development without GPU                                    |
| `universe-devel-cuda`           | Builds all universe sources with CUDA                                                | Development with GPU                                       |
| `universe-runtime-dependencies` | Runtime ansible roles + rosdep exec deps                                             | Foundation for final runtime images                        |
| `universe`                      | Runtime image with compiled autoware (no CUDA)                                       | Deployment without GPU                                     |
| `universe-cuda`                 | Runtime image with compiled autoware + CUDA runtime libs                             | Deployment with GPU                                        |

## Build locally

From the repository root. Targets beyond `base` require source repositories under `src/`:

```bash
# Clone source repositories (needed for core and universe targets)
vcs import src < repositories/autoware.repos

# Build all default targets (universe + universe-cuda)
docker buildx bake -f docker-new/docker-bake.hcl

# Build a specific target (dependencies are resolved automatically)
docker buildx bake -f docker-new/docker-bake.hcl base
docker buildx bake -f docker-new/docker-bake.hcl core-devel
docker buildx bake -f docker-new/docker-bake.hcl universe
docker buildx bake -f docker-new/docker-bake.hcl universe-cuda

# Build for humble
ROS_DISTRO=humble docker buildx bake -f docker-new/docker-bake.hcl base
```

## Usage

```bash
xhost +local:docker

docker run --rm -it \
  --net host \
  --privileged \
  --gpus all \
  -e DISPLAY=$DISPLAY \
  -e NVIDIA_DRIVER_CAPABILITIES=all \
  -e NVIDIA_VISIBLE_DEVICES=all \
  -e HOST_UID=$(id -u) \
  -e HOST_GID=$(id -g) \
  -e QT_X11_NO_MITSHM=1 \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  -v $HOME/autoware_map:/home/aw/autoware_map \
  -v $HOME/autoware_data:/home/aw/autoware_data \
  -v $HOME/autoware:/home/aw/autoware \
  -w /home/aw/autoware \
  --runtime=nvidia \
  autoware:universe-cuda-jazzy \
  bash -c "source /opt/autoware/setup.bash && exec bash"
```

| Flag                                | Why                                                                                                  |
| ----------------------------------- | ---------------------------------------------------------------------------------------------------- |
| `--rm`                              | Remove container on exit to avoid accumulating stopped containers                                    |
| `-it`                               | Interactive terminal (stdin + TTY)                                                                   |
| `--net host`                        | Share host network stack so ROS 2 nodes can discover each other                                      |
| `--privileged`                      | Access to host devices (sensors, CAN bus, etc.)                                                      |
| `--gpus all`                        | Expose all GPUs to the container                                                                     |
| `-e DISPLAY`                        | Forward X11 display for GUI applications (rviz2, rqt)                                                |
| `-e NVIDIA_DRIVER_CAPABILITIES=all` | Enable all NVIDIA driver features (compute, graphics, video)                                         |
| `-e NVIDIA_VISIBLE_DEVICES=all`     | Make all GPUs visible inside the container                                                           |
| `-e HOST_UID/HOST_GID`              | Entrypoint remaps the `aw` user to match host UID/GID, avoiding permission issues on mounted volumes |
| `-e QT_X11_NO_MITSHM`               | Disable MIT-SHM for Qt apps (shared memory doesn't work across container boundary)                   |
| `-v /tmp/.X11-unix`                 | Mount X11 socket for GUI forwarding                                                                  |
| `-v autoware_map`                   | Mount map data from host                                                                             |
| `-v autoware_data`                  | Mount perception model data from host                                                                |
| `-v autoware`                       | Mount source code for development                                                                    |
| `-w /home/aw/autoware`              | Set working directory to the mounted source                                                          |
| `--runtime=nvidia`                  | Use NVIDIA container runtime for GPU support                                                         |

Or run without volume mounting:

```bash
docker run --rm -it \
  --net host \
  autoware:core-jazzy
```

The default CycloneDDS config uses the `lo` interface (localhost only). To override it, mount your own config:

```bash
docker run --rm -it \
  --net host \
  -v /path/to/your/cyclonedds.xml:/home/aw/cyclonedds.xml \
  autoware:universe-cuda-jazzy
```
