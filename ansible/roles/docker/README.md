# pre_commit

This role installs Docker environment following [this page](https://docs.docker.com/engine/install/ubuntu/) and sets up rootless execution following [this page](https://docs.docker.com/engine/install/linux-postinstall/).

Also, it installs some additional tools:

- [Docker Compose](https://github.com/docker/compose)
- [NVIDIA Container Toolkit](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html#docker)
- [rocker](https://github.com/osrf/rocker)

## Inputs

| Name                   | Required | Description                    |
| ---------------------- | -------- | ------------------------------ |
| docker_compose_version | false    | The version of Docker Compose. |

## Manual Installation

Install Docker Engine and perform the post-installation steps:

```bash
# Taken from: https://docs.docker.com/engine/install/ubuntu/
# And: https://docs.docker.com/engine/install/linux-postinstall/

# Uninstall old versions
sudo apt-get remove docker docker-engine docker.io containerd runc

# Install using the repository
sudo apt-get update

sudo apt-get install \
    ca-certificates \
    curl \
    gnupg \
    lsb-release

# Add Docker’s official GPG key:
curl -fsSL https://download.docker.com/linux/ubuntu/gpg | sudo gpg --dearmor -o /usr/share/keyrings/docker-archive-keyring.gpg

# Install Docker Engine
sudo apt-get update
sudo apt-get install docker-ce docker-ce-cli containerd.io

# Verify that Docker Engine is installed correctly by running the hello-world image.
sudo docker run hello-world
# Note: This command downloads a test image and runs it in a container. When the container runs, it prints a message and exits.

# Post-installation steps for Linux

# Create the docker group.
sudo groupadd docker

# Add your user to the docker group.
sudo usermod -aG docker $USER

# Log out and log back in so that your group membership is re-evaluated.

# Verify that you can run docker commands without sudo
docker run hello-world
# Note: This command downloads a test image and runs it in a container. When the container runs, it prints a message and exits.
```

Install Nvidia Container Toolkit:

<!-- cspell:ignore Disp, Uncorr -->

```bash
# Taken from https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html#setting-up-nvidia-container-toolkit

# Setup the package repository and the GPG key:
distribution=$(. /etc/os-release;echo $ID$VERSION_ID) \
      && curl -s -L https://nvidia.github.io/libnvidia-container/gpgkey | sudo apt-key add - \
      && curl -s -L https://nvidia.github.io/libnvidia-container/$distribution/libnvidia-container.list | sudo tee /etc/apt/sources.list.d/nvidia-container-toolkit.list

# Install the nvidia-docker2 package (and dependencies) after updating the package listing:
sudo apt-get update
sudo apt-get install -y nvidia-docker2

# Restart the Docker daemon to complete the installation after setting the default runtime:
sudo systemctl restart docker

# At this point, a working setup can be tested by running a base CUDA container:
sudo docker run --rm --gpus all nvidia/cuda:11.0-base nvidia-smi

# This should result in a console output shown below:
# +-----------------------------------------------------------------------------+
# | NVIDIA-SMI 450.51.06    Driver Version: 450.51.06    CUDA Version: 11.0     |
# |-------------------------------+----------------------+----------------------+
# | GPU  Name        Persistence-M| Bus-Id        Disp.A | Volatile Uncorr. ECC |
# | Fan  Temp  Perf  Pwr:Usage/Cap|         Memory-Usage | GPU-Util  Compute M. |
# |                               |                      |               MIG M. |
# |===============================+======================+======================|
# |   0  Tesla T4            On   | 00000000:00:1E.0 Off |                    0 |
# | N/A   34C    P8     9W /  70W |      0MiB / 15109MiB |      0%      Default |
# |                               |                      |                  N/A |
# +-------------------------------+----------------------+----------------------+
#
# +-----------------------------------------------------------------------------+
# | Processes:                                                                  |
# |  GPU   GI   CI        PID   Type   Process name                  GPU Memory |
# |        ID   ID                                                   Usage      |
# |=============================================================================|
# |  No running processes found                                                 |
# +-----------------------------------------------------------------------------+
```

Install Docker Compose:

The `docker_compose_version` can also be found in:
[./defaults/main.yaml](./defaults/main.yaml)

```bash
# Modified from: https://docs.docker.com/compose/cli-command/#install-on-linux

# Run this command to download the Docker Compose:
docker_compose_version=v2.2.2
sudo curl -SL https://github.com/docker/compose/releases/download/${docker_compose_version}/docker-compose-$(uname -s)-$(uname -m) -o /usr/local/bin/docker-compose

# Apply executable permissions to the binary:
sudo chmod +x /usr/local/bin/docker-compose

# Test the installation.
docker-compose --version
```

Install rocker:

```bash
# Taken from: https://github.com/osrf/rocker#installation

# Add the ROS 2 apt repository to your system. First authorize our GPG key with apt.
sudo apt update && sudo apt install curl gnupg lsb-release
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

# Then add the repository to your sources list.
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(source /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

sudo apt update
sudo apt-get install python3-rocker
```
