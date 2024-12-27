# tensorrt

This role installs TensorRT and cuDNN following [the official NVIDIA TensorRT Installation Guide](https://docs.nvidia.com/deeplearning/tensorrt/install-guide/index.html#installing).

## Inputs

| Name             | Required | Description              |
| ---------------- | -------- | ------------------------ |
| cudnn_version    | true     | The version of cuDNN.    |
| tensorrt_version | true     | The version of TensorRT. |

## Manual Installation

### AMD64

```bash
# For the environment variables
wget -O /tmp/amd64.env https://raw.githubusercontent.com/autowarefoundation/autoware/main/amd64.env && source /tmp/amd64.env

sudo apt-get install -y \
libcudnn8=${cudnn_version} \
libnvinfer10=${tensorrt_version} \
libnvinfer-plugin10=${tensorrt_version} \
libnvonnxparsers10=${tensorrt_version} \
libcudnn8-dev=${cudnn_version} \
libnvinfer-dev=${tensorrt_version} \
libnvinfer-plugin-dev=${tensorrt_version} \
libnvinfer-headers-dev=${tensorrt_version} \
libnvinfer-headers-plugin-dev=${tensorrt_version} \
libnvonnxparsers-dev=${tensorrt_version}

sudo apt-mark hold \
libcudnn8 \
libnvinfer10 \
libnvinfer-plugin10 \
libnvonnxparsers10 \
libcudnn8-dev \
libnvinfer-dev \
libnvinfer-plugin-dev \
libnvonnxparsers-dev \
libnvinfer-headers-dev \
libnvinfer-headers-plugin-dev
```

### ARM64

```bash
# For the environment variables
wget -O /tmp/arm64.env https://raw.githubusercontent.com/autowarefoundation/autoware/main/arm64.env && source /tmp/arm64.env

sudo apt-get install -y \
libcudnn9-cuda-12=${cudnn_version} \
libnvinfer10=${tensorrt_version} \
libnvinfer-plugin10=${tensorrt_version} \
libnvonnxparsers10=${tensorrt_version} \
libcudnn9-dev-cuda-12=${cudnn_version} \
libnvinfer-dev=${tensorrt_version} \
libnvinfer-plugin-dev=${tensorrt_version} \
libnvinfer-headers-dev=${tensorrt_version} \
libnvinfer-headers-plugin-dev=${tensorrt_version} \
libnvonnxparsers-dev=${tensorrt_version}

sudo apt-mark hold \
libcudnn9-cuda-12 \
libnvinfer10 \
libnvinfer-plugin10 \
libnvonnxparsers10 \
libcudnn9-dev-cuda-12 \
libnvinfer-dev \
libnvinfer-plugin-dev \
libnvonnxparsers-dev \
libnvinfer-headers-dev \
libnvinfer-headers-plugin-dev
```
