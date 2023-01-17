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

# Modified from: https://developer.nvidia.com/cuda-downloads?target_os=Linux&target_arch=x86_64&Distribution=Ubuntu&target_version=22.04&target_type=deb_network

# A temporary workaround for Ubuntu 22.04 with the ubuntu2004 repository
if [[ "$(uname -m)" == "x86_64" ]]; then
  liburcu6_url=http://archive.ubuntu.com/ubuntu
else
  liburcu6_url=http://ports.ubuntu.com/ubuntu-ports
fi
sudo echo "deb $liburcu6_url focal main restricted" > /etc/apt/sources.list.d/focal.list

# TODO: Use 22.04 in https://github.com/autowarefoundation/autoware/pull/3084. Currently, 20.04 is intentionally used.
os=ubuntu2004
wget https://developer.download.nvidia.com/compute/cuda/repos/$os/$(uname -m)/cuda-keyring_1.0-1_all.deb
sudo dpkg -i cuda-keyring_1.0-1_all.deb
sudo apt-get update
cuda_version_dashed=$(eval sed -e "s/[.]/-/g" <<< "${cuda_version}")
sudo apt-get -y install cuda-${cuda_version_dashed}
```

Perform the post installation actions:

```bash
# Taken from: https://docs.nvidia.com/cuda/cuda-installation-guide-linux/index.html#post-installation-actions
echo 'export PATH=/usr/local/cuda/bin${PATH:+:${PATH}}' >> ~/.bashrc
echo 'export LD_LIBRARY_PATH=/usr/local/cuda/lib64${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}' >> ~/.bashrc
```
