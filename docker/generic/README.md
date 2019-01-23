Autoware Docker
===============

Docker can be used to allow developers to quickly get a development
environment ready to try and develop Autoware.

The main Docker images are:
* Base image - Provides a container with all the dependencies to build and
run Autoware. When creating a container using this image, the Autoware
source code is mounted as a volume allowing the user to develop and build
Autoware.
* Pre-built Autoware - Provides a container with a copy of Autoware
pre-built. This image is built on top of the base image.

When Cuda is enabled, a new base image is created which is then used to build
the pre-built Autoware image. Images containing Cuda support have the suffix
_-cuda_ allowing cuda and non-cuda images existing without conflict.

This set of Dockerfiles can be used to build and run containers natively on
both AArch64 and x86_64 systems.

Requirements
------------

* Recent version of [Docker CE](https://docs.docker.com/install/linux/docker-ce/ubuntu/)
* [NVIDIA Docker](https://github.com/NVIDIA/nvidia-docker) if your system
has Cuda support

How to build
------------

To build the docker image(s), use the build.sh script.
For details on the parameters available, try:

```
./build.sh --help
```

How to run
----------

To start a container use the run.sh tool. Which container will start
depends on the parameters passed to run.sh. For details on the parameters
available, try:

```
./run.sh --help
```

__Note__: The default values for the __--image__ and __--tag-prefix__
parameters in build.sh and run.sh are different. This is because run.sh
defaults to values used to retrieve images from Docker Hub. When running
containers from images you have built, make sure the parameters mentioned
match.
