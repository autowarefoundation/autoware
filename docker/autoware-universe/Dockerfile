FROM nvidia/cuda:11.4.1-devel-ubuntu20.04 as devel
SHELL ["/bin/bash", "-o", "pipefail", "-c"]

## Install apt packages
# hadolint ignore=DL3008
RUN apt-get update && DEBIAN_FRONTEND=noninteractive apt-get -y install --no-install-recommends \
  git \
  ssh \
  && apt-get clean \
  && rm -rf /var/lib/apt/lists/*

## Copy files
COPY autoware.repos setup-dev-env.sh ansible-galaxy-requirements.yaml /autoware/
COPY ansible/ /autoware/ansible/
WORKDIR /autoware
RUN ls /autoware

## Add GitHub to known hosts for private repositories
RUN mkdir -p ~/.ssh \
  && ssh-keyscan github.com >> ~/.ssh/known_hosts

## Set up development environment
RUN --mount=type=ssh \
  ./setup-dev-env.sh -y --no-nvidia universe \
  && mkdir src \
  && vcs import src < autoware.repos \
  && rosdep update \
  && DEBIAN_FRONTEND=noninteractive rosdep install -y --ignore-src --from-paths src --rosdistro galactic \
  && apt-get clean \
  && rm -rf /var/lib/apt/lists/*

## Install cuDNN and TensorRT
ARG CUDNN_VERSION=8.2.2.26-1+cuda11.4
ARG TENSORRT_VERSION=8.2.2-1+cuda11.4
RUN apt-get update && DEBIAN_FRONTEND=noninteractive apt-get -y install --no-install-recommends \
  libcudnn8=$CUDNN_VERSION \
  libcudnn8-dev=$CUDNN_VERSION \
  libnvinfer8=$TENSORRT_VERSION \
  libnvinfer-dev=$TENSORRT_VERSION \
  libnvinfer-plugin8=$TENSORRT_VERSION \
  libnvinfer-plugin-dev=$TENSORRT_VERSION \
  libnvonnxparsers8=$TENSORRT_VERSION \
  libnvonnxparsers-dev=$TENSORRT_VERSION \
  libnvparsers8=$TENSORRT_VERSION \
  libnvparsers-dev=$TENSORRT_VERSION \
  && apt-get clean \
  && rm -rf /var/lib/apt/lists/*

## Create entrypoint
# hadolint ignore=SC1091
RUN echo "source /opt/ros/galactic/setup.bash" > /etc/bash.bashrc
CMD ["/bin/bash"]

FROM devel as prebuilt
SHELL ["/bin/bash", "-o", "pipefail", "-c"]

## Build and change permission for runtime data conversion
# hadolint ignore=SC1091
RUN source /opt/ros/galactic/setup.bash \
  && colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release \
  && find /autoware/install -type d -exec chmod 777 {} \;

## Create entrypoint
# hadolint ignore=SC1091
RUN echo "source /autoware/install/setup.bash" > /etc/bash.bashrc
CMD ["/bin/bash"]
