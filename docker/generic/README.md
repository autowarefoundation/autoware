# Autoware Docker
Assuming the NVIDIA drivers and Docker and nvidia-docker (v2) are properly
installed.

[Docker installation](https://docs.docker.com/engine/installation/linux/docker-ce/ubuntu/)
[nvidia-docker installation](https://github.com/NVIDIA/nvidia-docker)

## How to build
```
$ cd Autoware/docker

# Ubuntu 14.04 (Indigo)
$ sh build.sh indigo nvidia # leave out the 'nvidia' argument if you want to build and intel or nvidia-docker v1 version

# Ubuntu 16.04 (Kinetic)
$ sh build.sh kinetic nvidia # leave out the 'nvidia' argument if you want to build and intel or nvidia-docker v1 version
```

## How to run
```
$ Default shared directory path is /home/$USER/shared_dir
$ sh run.sh

# If you select your shared directory path
$ sh run.sh {SHARED_DIR_PATH}
```
