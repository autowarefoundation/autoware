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
    --mount=type=bind,source=ansible/playbooks/install_nvidia.yaml,target=/tmp/ansible/ansible/playbooks/install_nvidia.yaml \
    --mount=type=cache,id=apt-cache-${ROS_DISTRO},target=/var/cache/apt,sharing=locked \
    --mount=type=cache,id=apt-lists-${ROS_DISTRO},target=/var/lib/apt/lists,sharing=locked \
    --mount=type=cache,id=pipx-cache,target=/home/aw/.cache/pipx,uid=1000,gid=1000 \
    pipx install --include-deps "ansible==10.*" && \
    cd /tmp/ansible && \
    ansible-galaxy collection install -f -r ansible-galaxy-requirements.yaml && \
    ansible-playbook autoware.dev_env.install_nvidia \
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
    --mount=type=bind,source=ansible/playbooks/install_nvidia.yaml,target=/tmp/ansible/ansible/playbooks/install_nvidia.yaml \
    --mount=type=cache,id=apt-cache-${ROS_DISTRO},target=/var/cache/apt,sharing=locked \
    --mount=type=cache,id=apt-lists-${ROS_DISTRO},target=/var/lib/apt/lists,sharing=locked \
    --mount=type=cache,id=pipx-cache,target=/home/aw/.cache/pipx,uid=1000,gid=1000 \
    pipx install --include-deps "ansible==10.*" && \
    cd /tmp/ansible && \
    ansible-galaxy collection install -f -r ansible-galaxy-requirements.yaml && \
    ansible-playbook autoware.dev_env.install_nvidia \
      -e install_devel=y \
      -e cuda_install_drivers=false && \
    pipx uninstall ansible
USER root

# CUDA 13 moved Thrust, CUB, and the libcudacxx `cuda/` headers under
# /usr/local/cuda/include/cccl/. Downstream consumers that still use the
# legacy unprefixed includes (e.g. `#include <thrust/sort.h>` in
# autoware_universe's bevdet_vendor) need both:
#   (a) `<thrust/...>` and `<cub/...>` to resolve — handled by relative
#       symlinks below;
#   (b) Thrust's INTERNAL chain to find `<cuda/__cccl_config>` etc., which
#       live at .../include/cccl/cuda/ — handled by adding the cccl include
#       root to CPATH so both nvcc and gcc/g++ pick it up transparently.
# Both are no-ops on CUDA 12.x: the cccl/ subdir doesn't exist there, the
# `for d` loop runs but creates nothing, and an unmatched CPATH entry is
# silently ignored by the compiler driver.
RUN for d in thrust cub; do \
      if [ -d "/usr/local/cuda/include/cccl/${d}" ] && [ ! -e "/usr/local/cuda/include/${d}" ]; then \
        ln -s "cccl/${d}" "/usr/local/cuda/include/${d}"; \
      fi; \
    done
ENV CPATH="/usr/local/cuda/include/cccl${CPATH:+:$CPATH}"
