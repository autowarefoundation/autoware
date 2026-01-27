# nvidia_container_toolkit

This role installs [NVIDIA Container Toolkit](https://github.com/NVIDIA/nvidia-container-toolkit) following the [installation guide](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html).

## Inputs

None.

## Manual Installation (Recommended)

> ℹ️ The steps below may differ from the role implementation.  
> They reflect the **most up-to-date and preferred procedure** for manual installation.  
> The role will be updated to align with these steps.

<!-- cspell:ignore Disp, Uncorr -->

```bash
# Install the prerequisites for the instructions below:
sudo apt-get update && sudo apt-get install -y --no-install-recommends \
   curl \
   gnupg2

# Configure the production repository:
curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey | sudo gpg --dearmor -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg \
  && curl -s -L https://nvidia.github.io/libnvidia-container/stable/deb/nvidia-container-toolkit.list | \
    sed 's#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#g' | \
    sudo tee /etc/apt/sources.list.d/nvidia-container-toolkit.list

# Update the packages list from the repository:
sudo apt-get update

# Install NVIDIA Container Toolkit
sudo apt-get install -y nvidia-container-toolkit

# Add NVIDIA runtime support to docker engine
sudo nvidia-ctk runtime configure --runtime=docker

# Restart docker daemon
sudo systemctl restart docker

# At this point, a working setup can be tested by running a base CUDA container:
sudo docker run --rm --gpus all nvcr.io/nvidia/cuda:12.8.1-runtime-ubuntu24.04 nvidia-smi

# ==========
# == CUDA ==
# ==========
#
# CUDA Version 12.8.1
#
# Container image Copyright (c) 2016-2023, NVIDIA CORPORATION & AFFILIATES. All rights reserved.
#
# This container image and its contents are governed by the NVIDIA Deep Learning Container License.
# By pulling and using the container, you accept the terms and conditions of this license:
# https://developer.nvidia.com/ngc/nvidia-deep-learning-container-license
#
# A copy of this license is made available in this container at /NGC-DL-CONTAINER-LICENSE for your convenience.
#
# Thu Jan 15 19:48:46 2026
# +-----------------------------------------------------------------------------------------+
# | NVIDIA-SMI 580.126.09             Driver Version: 580.126.09     CUDA Version: 13.0     |
# +-----------------------------------------+------------------------+----------------------+
# | GPU  Name                 Persistence-M | Bus-Id          Disp.A | Volatile Uncorr. ECC |
# | Fan  Temp   Perf          Pwr:Usage/Cap |           Memory-Usage | GPU-Util  Compute M. |
# |                                         |                        |               MIG M. |
# |=========================================+========================+======================|
# |   0  NVIDIA GeForce RTX 3090        On  |   00000000:2B:00.0  On |                  N/A |
# |  0%   52C    P8             41W /  350W |     369MiB /  24576MiB |     31%      Default |
# |                                         |                        |                  N/A |
# +-----------------------------------------+------------------------+----------------------+
#
# +-----------------------------------------------------------------------------------------+
# | Processes:                                                                              |
# |  GPU   GI   CI              PID   Type   Process name                        GPU Memory |
# |        ID   ID                                                               Usage      |
# |=========================================================================================|
# |  No running processes found                                                             |
# +-----------------------------------------------------------------------------------------+
```
