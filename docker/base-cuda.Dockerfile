# check=skip=InvalidDefaultArgInFrom
ARG BASE_IMAGE

FROM ${BASE_IMAGE} AS base-cuda-runtime
SHELL ["/bin/bash", "-o", "pipefail", "-c"]
ARG ROS_DISTRO

USER ${USERNAME}
# hadolint ignore=DL3003
RUN --mount=type=bind,source=ansible-galaxy-requirements.yaml,target=/tmp/ansible/ansible-galaxy-requirements.yaml \
    --mount=type=bind,source=ansible/galaxy.yml,target=/tmp/ansible/ansible/galaxy.yml \
    --mount=type=bind,source=ansible/roles/cuda,target=/tmp/ansible/ansible/roles/cuda \
    --mount=type=bind,source=ansible/roles/tensorrt,target=/tmp/ansible/ansible/roles/tensorrt \
    --mount=type=bind,source=ansible/roles/spconv,target=/tmp/ansible/ansible/roles/spconv \
    --mount=type=bind,source=ansible/playbooks/nvidia.yaml,target=/tmp/ansible/ansible/playbooks/nvidia.yaml \
    --mount=type=cache,id=apt-cache-${ROS_DISTRO},target=/var/cache/apt,sharing=locked \
    --mount=type=cache,id=apt-lists-${ROS_DISTRO},target=/var/lib/apt/lists,sharing=locked \
    --mount=type=cache,id=pipx-cache,target=/home/aw/.cache/pipx,uid=1000,gid=1000 \
    pipx install --include-deps "ansible==10.*" && \
    cd /tmp/ansible && \
    ansible-galaxy collection install -f -r ansible-galaxy-requirements.yaml && \
    ansible-playbook autoware.dev_env.nvidia \
      -e install_devel=N \
      -e cuda_install_drivers=false && \
    pipx uninstall ansible
USER root

ENV PATH="/usr/local/cuda/bin${PATH:+:$PATH}"
ENV LD_LIBRARY_PATH="/usr/local/cuda/lib64${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"

FROM base-cuda-runtime AS base-cuda-devel
ARG ROS_DISTRO

USER ${USERNAME}
# hadolint ignore=DL3003
RUN --mount=type=bind,source=ansible-galaxy-requirements.yaml,target=/tmp/ansible/ansible-galaxy-requirements.yaml \
    --mount=type=bind,source=ansible/galaxy.yml,target=/tmp/ansible/ansible/galaxy.yml \
    --mount=type=bind,source=ansible/roles/cuda,target=/tmp/ansible/ansible/roles/cuda \
    --mount=type=bind,source=ansible/roles/tensorrt,target=/tmp/ansible/ansible/roles/tensorrt \
    --mount=type=bind,source=ansible/roles/spconv,target=/tmp/ansible/ansible/roles/spconv \
    --mount=type=bind,source=ansible/playbooks/nvidia.yaml,target=/tmp/ansible/ansible/playbooks/nvidia.yaml \
    --mount=type=cache,id=apt-cache-${ROS_DISTRO},target=/var/cache/apt,sharing=locked \
    --mount=type=cache,id=apt-lists-${ROS_DISTRO},target=/var/lib/apt/lists,sharing=locked \
    --mount=type=cache,id=pipx-cache,target=/home/aw/.cache/pipx,uid=1000,gid=1000 \
    pipx install --include-deps "ansible==10.*" && \
    cd /tmp/ansible && \
    ansible-galaxy collection install -f -r ansible-galaxy-requirements.yaml && \
    ansible-playbook autoware.dev_env.nvidia \
      -e install_devel=y \
      -e cuda_install_drivers=false && \
    pipx uninstall ansible
USER root
