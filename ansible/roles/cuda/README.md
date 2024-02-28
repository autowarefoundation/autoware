# cuda

This role installs [CUDA Toolkit](https://developer.nvidia.com/cuda-toolkit) following [this page](https://developer.nvidia.com/cuda-downloads?target_os=Linux&target_arch=x86_64&Distribution=Ubuntu&target_version=22.04&target_type=deb_network) and [this page](https://docs.nvidia.com/cuda/cuda-installation-guide-linux/index.html#post-installation-actions).

This role also registers Vulkan, OpenGL, and OpenCL GPU vendors for future use.

## Inputs

| Name                 | Required | Description                      |
| -------------------- | -------- | -------------------------------- |
| cuda_version         | true     | The version of CUDA Toolkit.     |
| cuda_install_drivers | false    | Whether to install cuda-drivers. |

## Manual Installation

Follow these instructions to download and install the CUDA Toolkit and the corresponding NVIDIA Driver for Ubuntu 20.04.

For Universe, the `cuda_version` version can also be found in:
[../../playbooks/universe.yaml](../../playbooks/universe.yaml)

```bash
wget -O /tmp/amd64.env https://raw.githubusercontent.com/autowarefoundation/autoware/main/amd64.env && source /tmp/amd64.env

# From: https://docs.nvidia.com/cuda/cuda-installation-guide-linux/index.html#network-repo-installation-for-ubuntu

os=ubuntu2204
wget https://developer.download.nvidia.com/compute/cuda/repos/$os/$(uname -m)/cuda-keyring_1.1-1_all.deb
sudo dpkg -i cuda-keyring_1.1-1_all.deb
sudo apt-get update
cuda_version_dashed=$(eval sed -e "s/[.]/-/g" <<< "${cuda_version}")
sudo apt-get -y install cuda-${cuda_version_dashed}
```

Perform the post installation actions:

```bash
# Taken from: https://docs.nvidia.com/cuda/cuda-installation-guide-linux/index.html#post-installation-actions
echo 'export PATH=/usr/local/cuda/bin${PATH:+:${PATH}}' >> ~/.bashrc
echo 'export LD_LIBRARY_PATH=/usr/local/cuda/lib64${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}' >> ~/.bashrc

# Register Vulkan, OpenGL, and OpenCL GPU vendors

# Create Vulkan directory
sudo mkdir -p /etc/vulkan/icd.d
sudo chmod 0755 /etc/vulkan/icd.d

# Create OpenGL directory
sudo mkdir -p /etc/glvnd/egl_vendor.d
sudo chmod 0755 /etc/glvnd/egl_vendor.d

# Create OpenCL directory
sudo mkdir -p /etc/OpenCL/vendors
sudo chmod 0755 /etc/OpenCL/vendors

# Download and set permissions for Vulkan GPU vendors JSON
sudo wget https://gitlab.com/nvidia/container-images/vulkan/raw/dc389b0445c788901fda1d85be96fd1cb9410164/nvidia_icd.json -O /etc/vulkan/icd.d/nvidia_icd.json
sudo chmod 0644 /etc/vulkan/icd.d/nvidia_icd.json

# Download and set permissions for OpenGL GPU vendors JSON
sudo wget https://gitlab.com/nvidia/container-images/opengl/raw/5191cf205d3e4bb1150091f9464499b076104354/glvnd/runtime/10_nvidia.json -O /etc/glvnd/egl_vendor.d/10_nvidia.json
sudo chmod 0644 /etc/glvnd/egl_vendor.d/10_nvidia.json

# Register and set permissions for OpenCL GPU vendors
sudo touch /etc/OpenCL/vendors/nvidia.icd
echo "libnvidia-opencl.so.1" | sudo tee /etc/OpenCL/vendors/nvidia.icd > /dev/null
sudo chmod 0644 /etc/OpenCL/vendors/nvidia.icd
```
