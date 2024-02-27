# tensorrt

This role installs TensorRT and cuDNN following [this page](https://docs.nvidia.com/deeplearning/tensorrt/install-guide/index.html#installing).

## Inputs

| Name             | Required | Description              |
| ---------------- | -------- | ------------------------ |
| cudnn_version    | true     | The version of cuDNN.    |
| tensorrt_version | true     | The version of TensorRT. |

## Manual Installation

```bash
wget -O /tmp/amd64.env https://raw.githubusercontent.com/autowarefoundation/autoware/main/amd64.env && source /tmp/amd64.env

# Can also be found at: https://docs.nvidia.com/deeplearning/tensorrt/install-guide/index.html#installing

sudo apt-get install -y \
libcudnn8=${cudnn_version} \
libnvinfer8=${tensorrt_version} \
libnvinfer-plugin8=${tensorrt_version} \
libnvparsers8=${tensorrt_version} \
libnvonnxparsers8=${tensorrt_version} \

sudo apt-mark hold \
libcudnn8 \
libnvinfer8 \
libnvinfer-plugin8 \
libnvparsers8 \
libnvonnxparsers8

sudo apt-get install -y \
libcudnn8-dev=${cudnn_version} \
libnvinfer-dev=${tensorrt_version} \
libnvinfer-plugin-dev=${tensorrt_version} \
libnvparsers-dev=${tensorrt_version} \
libnvonnxparsers-dev=${tensorrt_version}

sudo apt-mark hold \
libcudnn8-dev \
libnvinfer-dev \
libnvinfer-plugin-dev \
libnvparsers-dev \
libnvonnxparsers-dev
```
