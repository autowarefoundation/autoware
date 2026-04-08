# Autoware on RHEL 9 (AlmaLinux 9): containerized build

This document describes how to build Autoware container images on AlmaLinux 9 (RHEL 9 compatible) using `docker/Dockerfile.rhel`.

## Key differences from Ubuntu build

| Aspect             | Ubuntu (`Dockerfile`)                | RHEL (`Dockerfile.rhel`)                             |
| ------------------ | ------------------------------------ | ---------------------------------------------------- |
| Base image         | Pre-built `$AUTOWARE_BASE_IMAGE`     | `almalinux:9` (from scratch)                         |
| Package manager    | apt-get                              | dnf                                                  |
| ROS 2 installation | Inherited from base image            | RHEL 9 binary tarball downloaded in `base` stage     |
| System setup       | `setup-dev-env.sh` (Ansible)         | Direct `dnf install`                                 |
| CUDA               | Supported (devel-cuda / cuda stages) | Not supported (COLCON_IGNORE)                       |
| Extra ROS packages | Inherited from base image            | `rhel-extra-deps.repos` (37 repos built from source) |

## Prerequisites

- Docker with BuildKit enabled
- Autoware source cloned with `vcs import`:

```bash
git clone https://github.com/autowarefoundation/autoware.git
cd autoware
mkdir src
vcs import src < autoware.repos
```

## Quick start

### Build development image (all packages + build tools)

```bash
cd /path/to/autoware
docker build \
  --progress=plain \
  --build-arg ROS_DISTRO=jazzy \
  --target universe-devel \
  -t autoware-rhel-universe-devel:latest \
  -f docker/Dockerfile.rhel \
  .
```

### Build runtime image (binaries only, no build tools)

```bash
docker build \
  --progress=plain \
  --build-arg ROS_DISTRO=jazzy \
  --target universe \
  -t autoware-rhel-universe:latest \
  -f docker/Dockerfile.rhel \
  .
```

### Run the container

```bash
# Runtime image
docker run -it --rm autoware-rhel-universe:latest

# Development image with volume mount
docker run -it --rm \
  -v $PWD/src/universe/autoware_universe/planning/autoware_path_optimizer:/autoware/src/autoware_path_optimizer \
  autoware-rhel-universe-devel:latest

# Runtime image with UID/GID remapping
docker run -it --rm \
  -e LOCAL_UID=$(id -u) -e LOCAL_USER=$(whoami) \
  -e LOCAL_GID=$(id -g) -e LOCAL_GROUP=$(id -gn) \
  -v /path/to/autoware_data:/autoware_data \
  autoware-rhel-universe:latest
```

## Build targets

| Target                  | Type        | Size     | Description                                                       |
| ----------------------- | ----------- | -------- | ----------------------------------------------------------------- |
| `universe-devel`        | Development | ~13.7 GB | All build tools, headers, source, and compiled artifacts          |
| `universe`              | Runtime     | ~7 GB    | Runtime libraries and binaries only                               |
| `core-devel`            | Development | —        | Core packages only (autoware_msgs, autoware_utils, autoware_core) |
| `core`                  | Runtime     | —        | Runtime image for core packages only                              |
| `universe-common-devel` | Development | —        | Core + common/external/middleware packages                        |

Specify the target with `--target`:

```bash
docker build --target core-devel -t autoware-rhel-core-devel -f docker/Dockerfile.rhel .
```

## Multi-stage Dockerfile structure

```
[base] almalinux:9 + ROS 2 Jazzy binary + dnf deps
  │
  ├─[source-deps] clone rhel-extra-deps.repos (37 ROS packages)
  │   │
  │   └─[rosdep-depend] resolve system deps for Autoware + extra deps
  │       ├─ /rosdep-all-depend-packages.txt (build + runtime)
  │       └─ /rosdep-exec-depend-packages.txt (runtime only)
  │
  ├─[core-common-devel] autoware_msgs, autoware_utils + deps_src
  │   │
  │   └─[core-devel] autoware_core packages
  │       │
  │       └─[universe-common-devel] external, common, middleware, launcher
  │           │
  │           ├─[universe-sensing-perception-devel]  ─┐
  │           ├─[universe-localization-mapping-devel]  │ BuildKit runs
  │           ├─[universe-planning-control-devel]      │ these 6 stages
  │           ├─[universe-vehicle-system-devel]        │ in parallel
  │           ├─[universe-visualization-devel]         │
  │           └─[universe-api-devel]                 ─┘
  │               │
  │               └─[universe-devel] COPY --from all 6 + strip + final build
  │
  └─[runtime-base] almalinux:9 + runtime libs only (no build tools)
      │
      ├─[core] COPY from core-devel + exec rosdep deps
      └─[universe] COPY from universe-devel + exec rosdep deps
```

### Stage descriptions

#### `base`

AlmaLinux 9 with EPEL and CRB repositories enabled. Installs system development libraries via `dnf`, downloads the [ROS 2 Jazzy RHEL 9 binary](https://github.com/ros2/ros2/releases) to `/opt/ros/jazzy/`, and applies RHEL-specific fixes (yaml-cpp CMake alias, magic_enum and png++ header-only libraries).

#### `source-deps`

Clones 37 ROS 2 packages from `rhel-extra-deps.repos` that are not included in the RHEL binary (which is ros-base equivalent, 371 packages). Selectively applies COLCON_IGNORE to sub-packages that cannot build on RHEL (e.g., rviz plugins, spacenav, wiimote).

#### `rosdep-depend`

Resolves rosdep keys to RHEL 9 system package names using `resolve_rosdep_keys_rhel.sh`. Generates two package lists: all dependencies (for devel stages) and exec-only dependencies (for runtime stages).

#### `core-common-devel`

Installs rosdep system packages via `dnf`. Copies extra dependency sources to `/autoware/deps_src/`. Applies runtime patches (M_PIf, CUDA COLCON_IGNORE, agnocast dedup). Builds core framework packages (autoware_msgs, autoware_utils, and all deps_src) to `/opt/autoware/`.

#### `core-devel`

Builds `autoware_core` packages on top of `core-common-devel`.

#### `universe-common-devel`

Builds common, external, middleware, and launcher packages.

#### `universe-{component}-devel`

Six parallel stages that each build a specific subsystem: sensing/perception, localization/mapping, planning/control, vehicle/system, visualization, and API.

#### `universe-devel`

Aggregates all six parallel stages via `COPY --from`, strips debug symbols from binaries, and runs a final build pass for remaining packages (evaluator, simulator, sensor_component).

#### `runtime-base`

Minimal AlmaLinux 9 image with only runtime libraries (no compilers, no `-devel` headers). Copies `/opt/ros/` from the `base` stage. Installs gosu for UID/GID remapping and Python runtime dependencies for `ros2` CLI.

#### `core` / `universe`

Runtime images that copy `/opt/autoware/` from corresponding devel stages and install exec-only rosdep dependencies. Broken ament_index entries (from packages that failed to build) are automatically cleaned up.

## RHEL-specific adaptations

### ROS 2 Jazzy RHEL binary

The official RHEL 9 binary provides ros-base equivalent (371 packages). Packages like `diagnostic_updater`, `geographic_msgs`, `pcl_ros`, `lanelet2_core`, `grid_map`, `filters`, etc. are not included and must be built from source via `rhel-extra-deps.repos`.

### Build script patches (`build_and_clean_rhel.sh`)

The build script applies runtime patches before each colcon build to handle RHEL 9 incompatibilities:

| Patch                                         | Reason                                                     |
| --------------------------------------------- | ---------------------------------------------------------- |
| `M_PIf` / `M_PI_2f` / `M_PI_4f` guards        | GCC 11 on RHEL 9 does not define C23 math float constants  |
| CUDA packages COLCON_IGNORE                   | 30 CUDA-dependent packages excluded (no CUDA toolkit)      |
| `tl/expected.hpp` warning suppression         | Deprecation `#warning` becomes error with `-Werror=cpp`    |
| Boost 1.75 `strategies/cartesian.hpp` include | `correct.hpp` and `is_valid.hpp` missing strategy includes |
| `boost::geometry::is_valid()` bypass          | Fails on custom Eigen-based Point2d with Boost < 1.77      |

The build uses a **three-pass retry strategy**: if packages fail, their `build/` directories are deleted and colcon retries on the next pass. This handles dependency ordering issues.

### Rosdep resolution (`resolve_rosdep_keys_rhel.sh`)

Uses `--os=rhel:9` to resolve against RHEL packages instead of Ubuntu defaults. Filters out `ros-jazzy-*` package names (no RPM repository for ROS on RHEL).

### yaml-cpp CMake alias

RHEL 9 ships yaml-cpp 0.6.3 which exports the CMake target `yaml-cpp`, but ROS 2 expects `yaml-cpp::yaml-cpp`. The `base` stage appends an alias to the CMake config file.

## Known limitations

- **No CUDA support**: All TensorRT/CUDA packages are excluded via COLCON_IGNORE. Perception nodes that require GPU inference will not be available.
- **Boost 1.75 constraints**: `boost::geometry::is_valid()` is bypassed with a simpler polygon size check for custom Point types. This may affect geometric validation accuracy in edge cases.
- **Build failures**: A small number of packages may fail to build (e.g., `autoware_control_evaluator` due to missing `boundary_departure_checker` header). These are automatically excluded from runtime images.
- **No setup-dev-env.sh**: The Ubuntu Ansible-based environment setup is replaced with direct `dnf install`. Some development tools available in the Ubuntu image may be missing.

## UID/GID remapping

The runtime images use `ros_entrypoint.sh` with gosu for UID/GID remapping. Set the following environment variables to run as a non-root user:

```bash
docker run -it --rm \
  -e LOCAL_UID=1000 -e LOCAL_USER=developer \
  -e LOCAL_GID=1000 -e LOCAL_GROUP=developers \
  -v /path/to/autoware_data:/autoware_data \
  autoware-rhel-universe:latest
```

If these variables are not set, the container runs as root.

## Files reference

| File                                         | Description                                                     |
| -------------------------------------------- | --------------------------------------------------------------- |
| `docker/Dockerfile.rhel`                     | Multi-stage Dockerfile (16 stages) for RHEL 9 build             |
| `docker/rhel-extra-deps.repos`               | VCS manifest of 37 ROS packages not in RHEL binary              |
| `docker/scripts/build_and_clean_rhel.sh`     | Colcon build orchestration with RHEL patches and 3-pass retry   |
| `docker/scripts/resolve_rosdep_keys_rhel.sh` | Rosdep resolution with `--os=rhel:9` and `ros-*` filtering      |
| `docker/scripts/cleanup_dnf.sh`              | DNF cache cleanup                                               |
| `docker/etc/ros_entrypoint.sh`               | Container entrypoint with UID/GID remapping via gosu            |
| `.dockerignore`                              | Build context filter (includes `!docker/rhel-extra-deps.repos`) |
