# Open AD Kit: containerized workloads for Autoware

[Open AD Kit](https://autoware.org/open-ad-kit/) offers containers for Autoware to simplify the development and deployment of Autoware and its dependencies. This directory contains scripts to build and run the containers.

Detailed instructions on how to use the containers can be found in the [Open AD Kit documentation](https://autowarefoundation.github.io/autoware-documentation/main/installation/autoware/docker-installation/).

## Multi-stage Dockerfile structure

![](./img/Dockerfile.svg)

### `$BASE_IMAGE`

This is a base image of this Dockerfile. [`ros:humble-ros-base-jammy`](https://hub.docker.com/_/ros/tags?page=&page_size=&ordering=&name=humble-ros-base-jammy) will be given.

### `base`

This stage performs only the basic setup required for all Autoware images.

### `rosdep-depend`

The following three ROS dependency package list files will be generated:

- `/rosdep-core-depend-packages.txt`: A dependency package list file for the packages under the `core` directory of `autoware.repos`.
- `/rosdep-universe-depend-packages.txt`: A dependency package list file for the packages under the `universe` directory of `autoware.repos`.
- `/rosdep-exec-depend-packages.txt`: A dependency package list file required for running Autoware.

These files will be used in the subsequent `autoware-core`, `autoware-universe`, and `runtime` stages.

By generating only the package list files and copying them to the subsequent stages, the dependency packages will not be reinstalled during the container build process unless the dependency packages change.

### `autoware-core`

This stage installs the dependency packages based on `/rosdep-core-depend-packages.txt` and build the packages under the `core` directory of `autoware.repos`.

### `autoware-universe`

This stage installs the dependency packages based on `/rosdep-universe-depend-packages.txt` and build the packages under the `universe` directory of `autoware.repos`.

### `devel`

This stage provides a [development container](https://containers.dev) to Autoware developers. By running the host's source code with volume mounting, it allows for easy building and debugging of Autoware.

### `runtime`

This stage is an Autoware runtime container. It only includes the dependencies given by `/rosdep-exec-depend-packages.txt`, the binaries built in the `autoware-universe` stage, and artifacts.
