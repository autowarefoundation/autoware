# ICURO Santa Clara, CA
# Authors: Jeremiah Liou, Arun Sunil, Cody Heiner

# Autoware Docker
To use the Autoware Docker, first make sure the Docker is properly installed.

[Docker installation](https://devtalk.nvidia.com/default/topic/1000224/jetson-tx2/docker-on-the-tx2/3): Post by merdzan

## How to Build
```
$ cd Autoware/docker/tx2

# Ubuntu 16.04 (Kinetic)
$ sh build
```

## How to Run
```
# Default shared directory path is /home/$USER/shared_dir

# Ubuntu 16.04 (Kinetic)
$ sh run
