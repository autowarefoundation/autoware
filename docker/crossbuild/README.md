# Autoware Cross Build Docker
To use the cross build tool, first make sure Docker is properly installed.

[Docker installation](https://docs.docker.com/engine/installation/linux/docker-ce/ubuntu/)

## How to Build Docker
Autoware users skip this step.
```
$ cd Autoware/docker/crossbuild/

# generic-aarch64
$ ./build_cross_image.sh generic-aarch64

# synquacer
$ ./build_cross_image.sh synquacer
```

## How to Build Cross
```
$ cd Autoware/ros/

# generic-aarch64
$ ./catkin_make_release_cross generic-aarch64

# synquacer
$ ./catkin_make_release_cross synquacer
```
