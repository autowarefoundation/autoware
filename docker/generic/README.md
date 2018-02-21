# Autoware Docker
Assuming the NVIDIA drivers and Docker and nvidia-docker are properly
installed.

[Docker installation](https://docs.docker.com/engine/installation/linux/docker-ce/ubuntu/)


[nvidia-docker installation](https://github.com/NVIDIA/nvidia-docker)

## How to build
```
$ cd Autoware/docker

# Ubuntu 14.04 (Indigo)
$ sh build.sh indigo

# Ubuntu 16.04 (Kinetic)
$ sh build.sh kinetic
```

## How to run
```
$ Default shared directory path is /home/$USER/shared_dir
$ sh run.sh

# If you select your shared directory path
$ sh run.sh {SHARED_DIR_PATH}
```
