# cuda

This role installs [CUDA Toolkit](https://developer.nvidia.com/cuda-toolkit) following [this page](https://developer.nvidia.com/cuda-downloads?target_os=Linux&target_arch=x86_64&Distribution=Ubuntu&target_version=20.04&target_type=deb_network) and [this page](https://docs.nvidia.com/cuda/cuda-installation-guide-linux/index.html#post-installation-actions).

## Inputs

| Name                 | Required | Description                      |
| -------------------- | -------- | -------------------------------- |
| cuda_version         | true     | The version of CUDA Toolkit.     |
| install_cuda_drivers | false    | Whether to install cuda-drivers. |

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
echo 'export PATH=/usr/local/cuda-12.3/bin${PATH:+:${PATH}}' >> ~/.bashrc
echo 'export LD_LIBRARY_PATH=/usr/local/cuda-12.3/lib64${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}' >> ~/.bashrc
```
