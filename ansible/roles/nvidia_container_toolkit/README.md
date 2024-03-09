# nvidia_container_toolkit

This role installs [NVIDIA Container Toolkit](https://github.com/NVIDIA/nvidia-container-toolkit) following the [installation guide](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html).

## Inputs

None.

## Manual Installation

Install Nvidia Container Toolkit:

<!-- cspell:ignore Disp, Uncorr -->

```bash
# Add NVIDIA container toolkit GPG key
sudo apt-key adv --fetch-keys https://nvidia.github.io/libnvidia-container/gpgkey
sudo gpg --no-default-keyring --keyring /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg --import /etc/apt/trusted.gpg

# Add NVIDIA container toolkit repository
echo "deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://nvidia.github.io/libnvidia-container/stable/deb/$(dpkg --print-architecture) /" | sudo tee /etc/apt/sources.list.d/nvidia-container-toolkit.list

# Update the package list
sudo apt-get update

# Install NVIDIA Container Toolkit
sudo apt-get install -y nvidia-container-toolkit

# Add NVIDIA runtime support to docker engine
sudo nvidia-ctk runtime configure --runtime=docker

# Restart docker daemon
sudo systemctl restart docker

# At this point, a working setup can be tested by running a base CUDA container:
sudo docker run --rm --gpus all nvcr.io/nvidia/cuda:12.3.1-runtime-ubuntu20.04 nvidia-smi

# This should result in a console output shown below:
# +-----------------------------------------------------------------------------+
# | NVIDIA-SMI 545.23.08    Driver Version: 545.23.08    CUDA Version: 12.3.1   |
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
