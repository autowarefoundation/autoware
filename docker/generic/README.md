# Autoware Docker
To use the Autoware Docker, first make sure the NVIDIA drivers, Docker and nvidia-docker are properly installed.

[Docker installation](https://docs.docker.com/engine/installation/linux/docker-ce/ubuntu/)


[nvidia-docker installation](https://github.com/NVIDIA/nvidia-docker)

## How to Build
```
$ cd Autoware/docker/generic/

# Ubuntu 14.04 (Indigo)
$ sh build.sh indigo

# Ubuntu 16.04 (Kinetic)
$ sh build.sh kinetic
```

## How to Run
```
# Default shared directory path is /home/$USER/shared_dir

# Ubuntu 14.04 (Indigo)
$ sh run.sh indigo

# Ubuntu 16.04 (Kinetic)
$ sh run.sh kinetic

# If you select your shared directory path
$ sh run.sh indigo|kinetic {SHARED_DIR_PATH}
```
