# Open AD Kit: containerized workloads for Autoware

[Open AD Kit](https://autoware.org/open-ad-kit/) offers containers for Autoware to simplify the development and deployment of Autoware and its dependencies. This directory contains scripts to build and run the containers.

Detailed instructions on how to use the containers can be found in the [Open AD Kit documentation](https://autowarefoundation.github.io/autoware-documentation/main/installation/autoware/docker-installation/).

## Multi-stage Dockerfile structure

![](./Dockerfile.svg)

The suffix `-devel` (e.g. `universe-devel`) is intended for use as a [development container](https://containers.dev). On the other hand, those without the `-devel` suffix (e.g. `universe`) are intended to be used as a runtime container.

### `$BASE_IMAGE`

This is a base image of this Dockerfile. [`ros:humble-ros-base-jammy`](https://hub.docker.com/_/ros/tags?page=&page_size=&ordering=&name=humble-ros-base-jammy) will be given.

### `base`

This stage performs only the basic setup required for all Autoware images.

### `rosdep-depend`

The ROS dependency package list files will be generated.
These files will be used in the subsequent stages:

- `core-devel`
- `universe-common`
- `universe-COMPONENT-devel` (e.g. `universe-sensing-perception-devel`)
- `universe-COMPONENT` (e.g. `universe-sensing-perception`)
- `universe-devel`
- `universe`

By generating only the package list files and copying them to the subsequent stages, the dependency packages will not be reinstalled during the container build process unless the dependency packages change.

### `core-devel`

This stage installs the dependency packages based on `/rosdep-core-depend-packages.txt` and build the packages under the `core` directory of `autoware.repos`.

### `universe-common-devel`

This stage installs the dependency packages based on `/rosdep-universe-common-depend-packages.txt` and build the packages under the following directories of `autoware.repos`.

- `universe/external`
- `universe/autoware.universe/common`

### `universe-sensing-perception-devel`

This stage installs the dependency packages based on `/rosdep-universe-sensing-perception-depend-packages.txt` and build the packages under the following directories of `autoware.repos`.

- `universe/autoware.universe/perception`
- `universe/autoware.universe/sensing`

### `universe-sensing-perception`

This stage is a Autoware Universe Sensing/Perception runtime container. It only includes the dependencies given by `/rosdep-universe-sensing-perception-exec-depend-packages.txt` and the binaries built in the `universe-sensing-perception-devel` stage.

### `universe-devel`

This stage installs the dependency packages based on `/rosdep-universe-depend-packages.txt` and build the remaining packages of `autoware.repos`:

- `launcher`
- `param`
- `sensor_component`
- `sensor_kit`
- `universe/autoware.universe/control`
- `universe/autoware.universe/evaluator`
- `universe/autoware.universe/launch`
- `universe/autoware.universe/localization`
- `universe/autoware.universe/map`
- `universe/autoware.universe/planning`
- `universe/autoware.universe/simulator`
- `universe/autoware.universe/system`
- `universe/autoware.universe/tools`
- `universe/autoware.universe/vehicle`
- `vehicle`

This stage provides an all-in-one development container to Autoware developers. By running the host's source code with volume mounting, it allows for easy building and debugging of Autoware.

### `universe`

This stage is an Autoware Universe runtime container. It only includes the dependencies given by `/rosdep-exec-depend-packages.txt`, the binaries built in the `universe-devel` stage, and artifacts.
