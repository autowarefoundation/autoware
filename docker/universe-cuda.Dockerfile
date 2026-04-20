# syntax=docker/dockerfile:1
# check=skip=InvalidDefaultArgInFrom,UndefinedVar
ARG BASE_CUDA_RUNTIME_IMAGE
ARG BASE_CUDA_DEVEL_IMAGE

FROM ${BASE_CUDA_DEVEL_IMAGE} AS universe-dependencies-cuda
SHELL ["/bin/bash", "-o", "pipefail", "-c"]
ARG ROS_DISTRO

USER ${USERNAME}
# hadolint ignore=DL3003
RUN --mount=type=bind,source=ansible-galaxy-requirements.yaml,target=/tmp/ansible/ansible-galaxy-requirements.yaml \
    --mount=type=bind,source=ansible,target=/tmp/ansible/ansible \
    --mount=type=cache,id=apt-cache-${ROS_DISTRO},target=/var/cache/apt,sharing=locked \
    --mount=type=cache,id=apt-lists-${ROS_DISTRO},target=/var/lib/apt/lists,sharing=locked \
    --mount=type=cache,id=pip-cache,target=/home/aw/.cache/pip,uid=1000,gid=1000 \
    --mount=type=cache,id=pipx-cache,target=/home/aw/.cache/pipx,uid=1000,gid=1000 \
    pipx install --include-deps "ansible==10.*" && \
    cd /tmp/ansible && \
    ansible-galaxy collection install -f -r ansible-galaxy-requirements.yaml && \
    ansible-playbook autoware.dev_env.autoware_requirements \
      --tags core,acados \
      --skip-tags base,nvidia \
      -e "rosdistro=${ROS_DISTRO}" && \
    sudo rm -rf /opt/acados/.git /opt/acados/examples /opt/acados/docs /opt/acados/test && \
    pipx uninstall ansible
USER root

ENV CC="/usr/lib/ccache/gcc"
ENV CXX="/usr/lib/ccache/g++"
ENV CCACHE_DIR="/home/aw/.ccache"
ENV CMAKE_PREFIX_PATH="/opt/acados${CMAKE_PREFIX_PATH:+:$CMAKE_PREFIX_PATH}"
ENV ACADOS_SOURCE_DIR="/opt/acados"
ENV LD_LIBRARY_PATH="/opt/acados/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"

# hadolint ignore=DL3022
COPY --from=autoware-core-devel /opt/autoware /opt/autoware

COPY --parents --chown=${USERNAME}:${USERNAME} src/**/package.xml /tmp/autoware/

RUN --mount=type=cache,id=apt-cache-${ROS_DISTRO},target=/var/cache/apt,sharing=locked \
    --mount=type=cache,id=apt-lists-${ROS_DISTRO},target=/var/lib/apt/lists,sharing=locked \
    apt-get update && \
    . "/opt/ros/${ROS_DISTRO}/setup.sh" && \
    . /opt/autoware/setup.sh && \
    rosdep install -y --from-paths /tmp/autoware/src \
      --ignore-src \
      --rosdistro "${ROS_DISTRO}" \
      --dependency-types=build \
      --dependency-types=build_export \
      --dependency-types=buildtool \
      --dependency-types=buildtool_export \
      --dependency-types=test && \
    rm -rf /tmp/autoware

FROM universe-dependencies-cuda AS universe-devel-cuda
ARG ROS_DISTRO

RUN --mount=type=bind,source=src,target=/tmp/autoware/src \
    --mount=type=cache,id=ccache-${ROS_DISTRO},target=/home/aw/.ccache,uid=1000,gid=1000,sharing=shared \
    . "/opt/ros/${ROS_DISTRO}/setup.sh" && \
    . /opt/autoware/setup.sh && \
    colcon build \
      --base-paths /tmp/autoware/src \
      --install-base /opt/autoware \
      --cmake-args -DCMAKE_BUILD_TYPE=Release && \
    rm -rf build log

FROM ${BASE_CUDA_RUNTIME_IMAGE} AS universe-cuda
ARG ROS_DISTRO
ENV AUTOWARE_RUNTIME=1

USER ${USERNAME}
# hadolint ignore=DL3003
RUN --mount=type=bind,source=ansible-galaxy-requirements.yaml,target=/tmp/ansible/ansible-galaxy-requirements.yaml \
    --mount=type=bind,source=ansible,target=/tmp/ansible/ansible \
    --mount=type=cache,id=apt-cache-${ROS_DISTRO},target=/var/cache/apt,sharing=locked \
    --mount=type=cache,id=apt-lists-${ROS_DISTRO},target=/var/lib/apt/lists,sharing=locked \
    --mount=type=cache,id=pip-cache,target=/home/aw/.cache/pip,uid=1000,gid=1000 \
    --mount=type=cache,id=pipx-cache,target=/home/aw/.cache/pipx,uid=1000,gid=1000 \
    sudo apt-get update && \
    pipx install --include-deps "ansible==10.*" && \
    cd /tmp/ansible && \
    ansible-galaxy collection install -f -r ansible-galaxy-requirements.yaml && \
    ansible-playbook autoware.dev_env.autoware_requirements \
      --tags geographiclib,qt5ct_setup \
      -e "rosdistro=${ROS_DISTRO}" && \
    pipx uninstall ansible
USER root

COPY --parents --chown=${USERNAME}:${USERNAME} src/**/package.xml /tmp/

RUN --mount=type=cache,id=apt-cache-${ROS_DISTRO},target=/var/cache/apt,sharing=locked \
    --mount=type=cache,id=apt-lists-${ROS_DISTRO},target=/var/lib/apt/lists,sharing=locked \
    apt-get update && \
    . "/opt/ros/${ROS_DISTRO}/setup.sh" && \
    sudo apt-get install -y --no-install-recommends "ros-${ROS_DISTRO}-topic-tools" && \
    rosdep install -y --from-paths /tmp/src \
      --dependency-types=exec \
      --ignore-src \
      --rosdistro "${ROS_DISTRO}" && \
    rm -rf /tmp/src

COPY --from=universe-devel-cuda /opt/acados /opt/acados
COPY --from=universe-devel-cuda /opt/autoware /opt/autoware
RUN find /opt/autoware -name '*.so' -exec strip --strip-unneeded {} +

ENV CMAKE_PREFIX_PATH="/opt/acados${CMAKE_PREFIX_PATH:+:$CMAKE_PREFIX_PATH}"
ENV ACADOS_SOURCE_DIR="/opt/acados"
ENV LD_LIBRARY_PATH="/opt/acados/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
