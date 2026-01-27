# cuda

This role installs [CUDA Toolkit](https://developer.nvidia.com/cuda-toolkit) following [this page](https://developer.nvidia.com/cuda-12-8-0-download-archive?target_os=Linux&target_arch=x86_64&Distribution=Ubuntu&target_version=22.04&target_type=deb_network) and [this page](https://docs.nvidia.com/cuda/cuda-installation-guide-linux/index.html#post-installation-actions).

This role also registers Vulkan, OpenGL, and OpenCL GPU vendors for future use.

## Inputs

| Name                 | Required | Description                      |
| -------------------- | -------- | -------------------------------- |
| cuda_version         | true     | The version of CUDA Toolkit.     |
| cuda_install_drivers | false    | Whether to install cuda-drivers. |

## Manual Installation

### Version compatibility

- CUDA version `12.8`.
- NVIDIA driver version `570` or **newer**.

#### 🛠️ For Advanced Users

⚠️ **Proceed with caution**: Avoid removing essential system components.

To prevent conflicts during NVIDIA installation, we recommend completely removing old NVIDIA Drivers and CUDA by following this guide: [How can I uninstall a NVIDIA driver completely?](https://askubuntu.com/a/206289/761440).

- **Important**: If you remove the previous NVIDIA drivers, ensure you install the recommended versions listed below **before restarting your system**.

Once the drivers are installed correctly, you may safely restart your system.

### CUDA Toolkit and Driver

Follow these instructions to download and install the CUDA Toolkit.

From: <https://docs.nvidia.com/cuda/cuda-installation-guide-linux/index.html#network-repo-installation-for-ubuntu>

```bash
wget -O /tmp/amd64.env https://raw.githubusercontent.com/autowarefoundation/autoware/main/amd64.env && source /tmp/amd64.env

os=ubuntu2204
arch_dir=$(
  case "$(dpkg --print-architecture)" in
    amd64) echo x86_64 ;;
    aarch64) echo arm64 ;;
    *) echo "$(dpkg --print-architecture)";;
  esac
)

wget "https://developer.download.nvidia.com/compute/cuda/repos/${os}/${arch_dir}/cuda-keyring_1.1-1_all.deb"
sudo dpkg -i cuda-keyring_1.1-1_all.deb
sudo apt-get update
cuda_version_dashed=$(eval sed -e "s/[.]/-/g" <<< "${cuda_version}")
sudo apt-get -y install cuda-toolkit-${cuda_version_dashed}
```

```bash
# ⚠️ this is the minimum version
sudo apt-get install -y cuda-drivers-570

# ✅ latest version is OK
apt search '^nvidia-driver-[0-9]+'
sudo apt install nvidia-driver-580  # or whichever is latest
```

Perform the post installation actions:

```bash
# Taken from: https://docs.nvidia.com/cuda/cuda-installation-guide-linux/index.html#post-installation-actions
echo 'export PATH=/usr/local/cuda/bin${PATH:+:${PATH}}' >> ~/.bashrc
echo 'export LD_LIBRARY_PATH=/usr/local/cuda/lib64${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}' >> ~/.bashrc
```

### GPU Vendors

Register Vulkan, OpenGL, and OpenCL GPU vendors following the instructions below.

```bash
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
