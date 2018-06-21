# Autoware Docker
To use the Autoware Docker, first make sure Docker is properly installed.

[Docker installation](https://docs.docker.com/install/linux/docker-ce/debian/)

## How to Build
```
$ cd Autoware/docker/96boards/

# Ubuntu 16.04 (Kinetic)
$ sh build.sh kinetic
```

## How to Run
```
# Default shared directory path is /home/$USER/shared_dir

# Ubuntu 16.04 (Kinetic)
$ sh run.sh kinetic

# If you select your shared directory path
$ sh run.sh kinetic {SHARED_DIR_PATH}
```
