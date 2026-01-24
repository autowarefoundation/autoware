# tensorrt

This role installs TensorRT following [the official NVIDIA TensorRT Installation Guide](https://docs.nvidia.com/deeplearning/tensorrt/install-guide/index.html#installing).

## Inputs

| Name             | Required | Description              |
| ---------------- | -------- | ------------------------ |
| tensorrt_version | true     | The version of TensorRT. |

## Manual Installation

### Set up the environment variables

Choose **one** ROS distribution and run the corresponding command.

#### ROS 2 Humble

```bash
wget -O /tmp/amd64.env https://raw.githubusercontent.com/autowarefoundation/autoware/main/amd64.env && \
source /tmp/amd64.env
```

#### ROS 2 Jazzy

```bash
wget -O /tmp/amd64.env https://raw.githubusercontent.com/autowarefoundation/autoware/main/amd64_jazzy.env && \
source /tmp/amd64.env
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
