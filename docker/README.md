# Open AD Kit: containerized workloads for Autoware

[Open AD Kit](https://autoware.org/open-ad-kit/) offers containers for Autoware to simplify the development and deployment of Autoware and its dependencies. This directory contains scripts to build and run the containers.

Detailed instructions on how to use the containers can be found in the [Open AD Kit documentation](https://autowarefoundation.github.io/autoware-documentation/main/installation/autoware/docker-installation/).

## Multi-stage Dockerfile structure

![](./img/Dockerfile.svg)

The suffix `-devel` (e.g. `universe-devel`) is intended for use as a development container. On the other hand, those without the `-devel` suffix (e.g. `universe`) are intended to be used as a runtime container.

### `$BASE_IMAGE`

This is a base image of this Dockerfile. [`ros:humble-ros-base-jammy`](https://hub.docker.com/_/ros/tags?page=&page_size=&ordering=&name=humble-ros-base-jammy) will be given.

### `base`

This stage performs only the basic setup required for all Autoware images.

### `rosdep-depend`

The ROS dependency package list files will be generated.
These files will be used in the subsequent `core-devel`, `universe-COMPONENT-devel` `universe-COMPONENT`, `universe-devel`, and `universe` stages.

By generating only the package list files and copying them to the subsequent stages, the dependency packages will not be reinstalled during the container build process unless the dependency packages change.

### `core-devel`

This stage installs the dependency packages based on `/rosdep-core-depend-packages.txt` and build the packages under the `core` directory of `autoware.repos`.

### `universe-common-devel`

This stage installs the dependency packages based on `/rosdep-universe-common-depend-packages.txt` and build the packages under the following directories of `autoware.repos`.

- `universe/external`
- `universe/autoware.universe/common`

### `autoware-universe`

This stage installs the dependency packages based on `/rosdep-universe-depend-packages.txt` and build the packages under the `universe` directory of `autoware.repos`.

### `devel`

This stage provides a [development container](https://containers.dev) to Autoware developers. By running the host's source code with volume mounting, it allows for easy building and debugging of Autoware.

### `runtime`

This stage is an Autoware runtime container. It only includes the dependencies given by `/rosdep-exec-depend-packages.txt`, the binaries built in the `autoware-universe` stage, and artifacts.
