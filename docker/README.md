# Run Autoware in Docker

<!-- cspell:ignore libcuda libcudart ctypes CDLL byref prereq tegrastats NVDEC NVENC -->

## Image Graph

```mermaid
graph TB
    base(["base"]) --> core-dependencies(["core-dependencies"]) & core(["core"]) & base-cuda-runtime(["base-cuda-runtime"])
    core-dependencies --> core-devel(["core-devel"])
    core-devel --> universe-dependencies(["universe-dependencies"])
    universe-dependencies --> universe-devel(["universe-devel"])
    universe-dependencies-cuda(["universe-dependencies-cuda"]) --> universe-devel-cuda(["universe-devel-cuda"])
    core-devel -- " COPY /opt/autoware " --> core
    core --> universe(["universe"])
    universe-devel -- " COPY /opt/autoware " --> universe
    universe-devel-cuda -- " COPY /opt/autoware " --> universe-cuda(["universe-cuda"])
    core-devel -- " COPY /opt/autoware " --> universe-dependencies-cuda
    base-cuda-devel(["base-cuda-devel"]) --> universe-dependencies-cuda
    base-cuda-runtime --> universe-cuda & base-cuda-devel
    classDef base fill: #e8e8e8, color: #333
    classDef devel fill: #bbdefb, color: #333
    classDef runtime fill: #c8e6c9, color: #333
    classDef cuda fill: #e1bee7, color: #333
    class base,base-cuda-runtime,base-cuda-devel base
    class core-dependencies,core-devel,universe-dependencies,universe-devel devel
    class core,universe runtime
    class universe-dependencies-cuda,universe-devel-cuda,universe-cuda cuda
```

## Images

| Image                        | Description                                                                           | Use case                                                   |
| ---------------------------- | ------------------------------------------------------------------------------------- | ---------------------------------------------------------- |
| `base`                       | ROS base, sudo, pipx, ansible, RMW, user `aw`                                         | Foundation for all other images                            |
| `core-dependencies`          | Build deps + compiled core packages (except autoware_core and autoware_rviz_plugins)  | CI for autoware_core                                       |
| `core-devel`                 | Adds autoware_core build on top of core-dependencies                                  | Development and CI for packages depending on autoware_core |
| `core`                       | Runtime-only: rosdep exec deps + compiled core from core-devel                        | Lightweight core runtime                                   |
| `base-cuda-runtime`          | `base` + CUDA/cuDNN/TensorRT runtime libs (no `-dev` packages)                        | Runtime foundation for `universe-cuda`                     |
| `base-cuda-devel`            | `base-cuda-runtime` + CUDA/cuDNN/TensorRT dev headers                                 | Build foundation for CUDA universe packages                |
| `universe-dependencies`      | Acados + rosdep build deps for all of autoware (non-CUDA path)                        | CI for autoware_universe                                   |
| `universe-dependencies-cuda` | Core ansible roles + acados + rosdep build deps, inherits CUDA from `base-cuda-devel` | CI for CUDA-dependent packages                             |
| `universe-devel`             | Builds all universe sources (no CUDA)                                                 | Development without GPU                                    |
| `universe-devel-cuda`        | Builds all universe sources with CUDA                                                 | Development with GPU                                       |
| `universe`                   | Runtime image with compiled autoware (no CUDA)                                        | Deployment without GPU                                     |
| `universe-cuda`              | Runtime image with compiled autoware, inherits CUDA runtime from `base-cuda-runtime`  | Deployment with GPU                                        |

## Pull from GHCR

Pre-built multi-arch images (amd64 + arm64) are available on GHCR:

```bash
# Pull a specific image
docker pull ghcr.io/autowarefoundation/autoware:base-jazzy
docker pull ghcr.io/autowarefoundation/autoware:base-humble

# Pull a dated version (for pinning)
docker pull ghcr.io/autowarefoundation/autoware:base-jazzy-20260407

# Pull a release version
docker pull ghcr.io/autowarefoundation/autoware:base-jazzy-1.2.3
```

Tag pattern: `<stage>-<ros_distro>[-<date>|-<version>]`

Available images (replace `jazzy` with `humble` for other distros):

| Tag                                | Description                                       |
| ---------------------------------- | ------------------------------------------------- |
| `base-jazzy`                       | ROS base + ansible + user aw                      |
| `core-dependencies-jazzy`          | Build deps + core packages (except autoware_core) |
| `core-devel-jazzy`                 | Full core development image                       |
| `core-jazzy`                       | Lightweight core runtime                          |
| `base-cuda-runtime-jazzy`          | base + CUDA/cuDNN/TensorRT runtime                |
| `base-cuda-devel-jazzy`            | base-cuda-runtime + CUDA/cuDNN/TensorRT dev       |
| `universe-dependencies-jazzy`      | Universe build dependencies (no CUDA)             |
| `universe-dependencies-cuda-jazzy` | Universe build deps on base-cuda-devel            |
| `universe-devel-jazzy`             | Full universe development (no CUDA)               |
| `universe-devel-cuda-jazzy`        | Full universe development with CUDA               |
| `universe-jazzy`                   | Runtime without GPU                               |
| `universe-cuda-jazzy`              | Runtime with GPU                                  |

## Build locally

From the repository root. Targets beyond `base` require source repositories under `src/`:

```bash
# Clone source repositories (needed for core and universe targets)
vcs import src < repositories/autoware.repos

# Build all default targets (universe + universe-cuda)
docker buildx bake -f docker/docker-bake.hcl

# Build a specific target (dependencies are resolved automatically)
docker buildx bake -f docker/docker-bake.hcl base
docker buildx bake -f docker/docker-bake.hcl core-devel
docker buildx bake -f docker/docker-bake.hcl universe
docker buildx bake -f docker/docker-bake.hcl universe-cuda

# Build for humble
ROS_DISTRO=humble docker buildx bake -f docker/docker-bake.hcl base
```

## Reproducible builds (`USE_LOCKFILE`)

By default a target builds against the floating `ros:<distro>-ros-base` tag and the latest packages from `packages.ros.org` and the Ubuntu archive, so the same target built on different days can differ. Set `USE_LOCKFILE=true` to make the build reproducible:

```bash
# Reproducible base image for the default distro (jazzy)
USE_LOCKFILE=true docker buildx bake -f docker/docker-bake.hcl base

# Reproducible build for humble
USE_LOCKFILE=true ROS_DISTRO=humble docker buildx bake -f docker/docker-bake.hcl base
```

When enabled, the build:

- pins the base image to the per-distro digest in `BASE_IMAGE_DIGESTS` (`docker/docker-bake.hcl`) instead of the floating `ros:<distro>-ros-base` tag, freezing the OS-level closure;
- passes `use_locked_versions=true` to ansible, which pins Ubuntu-archive packages to the versions in `ansible/vars/locked-versions-<distro>-<arch>.yaml` and points ROS at the dated `snapshots.ros.org` source recorded in that lockfile, freezing the entire ROS dependency closure; and
- verifies after install that every locked package landed at its pinned version, failing the build on any drift.

Only distros that have **both** a lockfile and a `BASE_IMAGE_DIGESTS` entry can be built in locked mode (currently `humble` and `jazzy`); requesting `USE_LOCKFILE=true` for any other `ROS_DISTRO` fails the build rather than silently falling back to a floating base. The CUDA targets (`base-cuda-*`, `universe-cuda`) are intentionally not locked: they inherit the frozen apt pins and dated snapshot from the `base` image they build on, but the CUDA / cuDNN / TensorRT packages from NVIDIA's apt repositories are not yet covered by the lockfile.

See [`ansible/roles/version_lock/README.md`](../ansible/roles/version_lock/README.md) for the lockfile format and how to generate or update lockfiles.

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
  -v $HOME/autoware_data/maps:/home/aw/autoware_data/maps \
  -v $HOME/autoware_data/ml_models:/home/aw/autoware_data/ml_models \
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
| `-v autoware_data/maps`             | Mount maps from host (lanelet2 + pointcloud)                                                         |
| `-v autoware_data/ml_models`        | Mount ML model artifacts from host                                                                   |
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

### Running on NVIDIA Thor (Jetson Thor / DRIVE Thor)

Under JetPack 7's "unified CUDA 13.0 across all Arm targets" the `universe-cuda-jazzy` image built for `linux/arm64` (SBSA flavor) is the supported runtime on **both Thor variants** — Jetson Thor running JetPack 7 and DRIVE Thor running DRIVE OS share the same Blackwell SoC (`sm_110`) and the same SBSA-compatible CUDA / TensorRT package set.

> The fixes and runtime steps below have been verified end-to-end on a local Jetson Thor (L4T R38.4.0 / CUDA 13.0). DRIVE Thor is expected to work by design — the in-container CUDA stack is identical — but its host-side container-toolkit packaging comes via DRIVE OS rather than the JetPack BSP, so the prereq command names below may differ on a DRIVE host.

Prerequisites on the Thor host:

- An Arm host with Ubuntu 24.04 + Linux 6.8 (Jetson Thor on JetPack 7, or a DRIVE Thor target running DRIVE OS).
- `nvidia-container-toolkit` from the JetPack BSP (or the DRIVE OS equivalent):

  ```bash
  sudo apt-get install -y nvidia-container-toolkit
  sudo nvidia-ctk runtime configure --runtime=docker
  sudo systemctl restart docker
  ```

Launching the image:

```bash
docker run --rm -it \
  --net host \
  --runtime nvidia \
  -e NVIDIA_VISIBLE_DEVICES=all \
  -e NVIDIA_DRIVER_CAPABILITIES=all \
  -e HOST_UID=$(id -u) -e HOST_GID=$(id -g) \
  -v $HOME/autoware_data/maps:/home/aw/autoware_data/maps \
  -v $HOME/autoware_data/ml_models:/home/aw/autoware_data/ml_models \
  autoware:universe-cuda-jazzy \
  bash -c "source /opt/autoware/setup.bash && exec bash"
```

Notes:

- `--gpus all` is **not** used on Tegra; `--runtime nvidia` plus the two `NVIDIA_*` env vars is the supported path. The env vars are what trigger the L4T container toolkit to mount `libcuda.so.1` and `/dev/nvidia-*` into the container — without them `libcuda.so` is absent and CUDA calls fail with `cudaErrorInsufficientDriver` (35).
- `--privileged` is omitted relative to the generic Usage example above; the Thor inference workload here does not touch sensors or CAN bus. Re-add it if you wire in external hardware that needs host device access.
- Inside the container, verify the GPU is reachable:

  ```bash
  python3 -c '
  import ctypes
  rt = ctypes.CDLL("libcudart.so")
  cnt = ctypes.c_int(); rt.cudaGetDeviceCount(ctypes.byref(cnt))
  maj = ctypes.c_int(); minor = ctypes.c_int()
  rt.cudaDeviceGetAttribute(ctypes.byref(maj), 75, 0)
  rt.cudaDeviceGetAttribute(ctypes.byref(minor), 76, 0)
  print(f"devices={cnt.value}, sm_{maj.value}{minor.value}")
  '
  ```

  On Thor this prints `devices=1, sm_110` (rather than a PTX JIT fallback or an `Insufficient driver` error).

- On the host, `sudo tegrastats` while inference runs shows iGPU utilization (`GR3D_FREQ`) climbing above idle.

DLA, VPI, NVDEC/NVENC, and Argus camera support are intentionally out of scope for this image; they require a separate L4T-derived variant.
