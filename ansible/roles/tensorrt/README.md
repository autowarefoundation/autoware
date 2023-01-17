# tensorrt

This role installs TensorRT and cuDNN following [this page](https://docs.nvidia.com/deeplearning/tensorrt/install-guide/index.html#installing).

## Inputs

| Name             | Required | Description              |
| ---------------- | -------- | ------------------------ |
| cudnn_version    | true     | The version of cuDNN.    |
| tensorrt_version | true     | The version of TensorRT. |

## Manual Installation

For Universe, the `cudnn_version` and `tensorrt_version` variables should be copied from
[amd64.env](../../../amd64.env) or [arm64.env](../../../arm64.env) depending on the architecture used.

```bash
wget -O /tmp/amd64.env https://raw.githubusercontent.com/autowarefoundation/autoware/main/amd64.env && source /tmp/amd64.env

# Taken from: https://docs.nvidia.com/deeplearning/tensorrt/install-guide/index.html#installing

sudo apt-get install libcudnn8=${cudnn_version} libcudnn8-dev=${cudnn_version}
sudo apt-mark hold libcudnn8 libcudnn8-dev

sudo apt-get install libnvinfer8=${tensorrt_version} libnvonnxparsers8=${tensorrt_version} libnvparsers8=${tensorrt_version} libnvinfer-plugin8=${tensorrt_version} libnvinfer-dev=${tensorrt_version} libnvonnxparsers-dev=${tensorrt_version} libnvparsers-dev=${tensorrt_version} libnvinfer-plugin-dev=${tensorrt_version}
sudo apt-mark hold libnvinfer8 libnvonnxparsers8 libnvparsers8 libnvinfer-plugin8 libnvinfer-dev libnvonnxparsers-dev libnvparsers-dev libnvinfer-plugin-dev
```
