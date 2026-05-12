# tensorrt

This role installs TensorRT following [the official NVIDIA TensorRT Installation Guide](https://docs.nvidia.com/deeplearning/tensorrt/install-guide/index.html#installing).

## Inputs

| Name             | Required | Description              |
| ---------------- | -------- | ------------------------ |
| tensorrt_version | true     | The version of TensorRT. |

## Version selection

| Ubuntu         | Architecture                               | TensorRT version     |
| -------------- | ------------------------------------------ | -------------------- |
| 22.04 (humble) | x86_64                                     | 10.8.0.43-1+cuda12.8 |
| 22.04 (humble) | aarch64 (SBSA)                             | 10.3.0.26-1+cuda12.5 |
| 24.04 (jazzy)  | x86_64 / aarch64 (SBSA, incl. Jetson Thor) | 10.13.3.9-1+cuda13.0 |

Override `tensorrt_version` explicitly via `-e tensorrt_version=…` to pin a different release.

## Manual Installation

### Set up the environment variables

```bash
# From the Autoware repository root:
# defaults/main.yaml contains an architecture-dependent Jinja2 expression;
# extract the matching version for this machine.
if [ "$(uname -m)" = "aarch64" ]; then
  tensorrt_version=$(grep -oP "'\K[^']+(?=' if)" ansible/roles/tensorrt/defaults/main.yaml)
else
  tensorrt_version=$(grep -oP "else '\K[^']+" ansible/roles/tensorrt/defaults/main.yaml)
fi
```

### Install TensorRT

```bash
sudo apt-get install -y \
libnvinfer10=${tensorrt_version} \
libnvinfer-plugin10=${tensorrt_version} \
libnvonnxparsers10=${tensorrt_version} \
libnvinfer-dev=${tensorrt_version} \
libnvinfer-plugin-dev=${tensorrt_version} \
libnvinfer-headers-dev=${tensorrt_version} \
libnvinfer-headers-plugin-dev=${tensorrt_version} \
libnvonnxparsers-dev=${tensorrt_version}

sudo apt-mark hold \
libnvinfer10 \
libnvinfer-plugin10 \
libnvonnxparsers10 \
libnvinfer-dev \
libnvinfer-plugin-dev \
libnvonnxparsers-dev \
libnvinfer-headers-dev \
libnvinfer-headers-plugin-dev
```
