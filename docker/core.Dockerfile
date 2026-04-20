# syntax=docker/dockerfile:1
# check=skip=InvalidDefaultArgInFrom
ARG BASE_IMAGE

FROM ${BASE_IMAGE} AS core-dependencies
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
      --tags core \
      --skip-tags base \
      -e "rosdistro=${ROS_DISTRO}" && \
    pipx uninstall ansible
USER root

ENV CC="/usr/lib/ccache/gcc"
ENV CXX="/usr/lib/ccache/g++"
ENV CCACHE_DIR="/home/aw/.ccache"

COPY --parents --chown=${USERNAME}:${USERNAME} src/core/**/package.xml /tmp/autoware/
RUN rm -rf /tmp/autoware/src/core/autoware_core /tmp/autoware/src/core/autoware_rviz_plugins

RUN --mount=type=cache,id=apt-cache-${ROS_DISTRO},target=/var/cache/apt,sharing=locked \
    --mount=type=cache,id=apt-lists-${ROS_DISTRO},target=/var/lib/apt/lists,sharing=locked \
    apt-get update && \
    . "/opt/ros/${ROS_DISTRO}/setup.sh" && \
    rosdep install -y --from-paths /tmp/autoware/src/core \
      --ignore-src \
      --rosdistro "${ROS_DISTRO}" \
      --dependency-types=build \
      --dependency-types=build_export \
      --dependency-types=buildtool \
      --dependency-types=buildtool_export \
      --dependency-types=test

RUN --mount=type=bind,source=src/core,target=/tmp/autoware/src/core,rw \
    --mount=type=cache,id=ccache-${ROS_DISTRO},target=/home/aw/.ccache,uid=1000,gid=1000 \
    rm -rf /tmp/autoware/src/core/autoware_core \
           /tmp/autoware/src/core/autoware_rviz_plugins && \
    . "/opt/ros/${ROS_DISTRO}/setup.sh" && \
    colcon build \
      --base-paths /tmp/autoware/src/core \
      --install-base /opt/autoware \
      --cmake-args -DCMAKE_BUILD_TYPE=Release && \
    rm -rf build log

FROM core-dependencies AS core-devel
ARG ROS_DISTRO

COPY --parents --chown=${USERNAME}:${USERNAME} \
    src/core/autoware_core/**/package.xml \
    src/core/autoware_rviz_plugins/**/package.xml \
    /tmp/autoware/

RUN --mount=type=cache,id=apt-cache-${ROS_DISTRO},target=/var/cache/apt,sharing=locked \
    --mount=type=cache,id=apt-lists-${ROS_DISTRO},target=/var/lib/apt/lists,sharing=locked \
    apt-get update && \
    . "/opt/ros/${ROS_DISTRO}/setup.sh" && \
    . /opt/autoware/setup.sh && \
    rosdep install -y --from-paths /tmp/autoware/src/core \
      --ignore-src \
      --rosdistro "${ROS_DISTRO}" \
      --dependency-types=build \
      --dependency-types=build_export \
      --dependency-types=buildtool \
      --dependency-types=buildtool_export \
      --dependency-types=test

RUN --mount=type=bind,source=src/core/autoware_core,target=/tmp/autoware/src/core/autoware_core \
    --mount=type=bind,source=src/core/autoware_rviz_plugins,target=/tmp/autoware/src/core/autoware_rviz_plugins \
    --mount=type=cache,id=ccache-${ROS_DISTRO},target=/home/aw/.ccache,uid=1000,gid=1000 \
    . "/opt/ros/${ROS_DISTRO}/setup.sh" && \
    . /opt/autoware/setup.sh && \
    colcon build \
      --base-paths /tmp/autoware/src/core/autoware_core \
                   /tmp/autoware/src/core/autoware_rviz_plugins \
      --install-base /opt/autoware \
      --cmake-args -DCMAKE_BUILD_TYPE=Release && \
    rm -rf build log

FROM ${BASE_IMAGE} AS core
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

COPY --from=core-devel /opt/autoware /opt/autoware
RUN find /opt/autoware -name '*.so' -exec strip --strip-unneeded {} +

COPY --parents src/core/**/package.xml /tmp/

RUN --mount=type=cache,id=apt-cache-${ROS_DISTRO},target=/var/cache/apt,sharing=locked \
    --mount=type=cache,id=apt-lists-${ROS_DISTRO},target=/var/lib/apt/lists,sharing=locked \
    apt-get update && \
    . "/opt/ros/${ROS_DISTRO}/setup.sh" && \
    sudo apt-get install -y "ros-${ROS_DISTRO}-topic-tools" && \
    rosdep install -y --from-paths /tmp/src/core \
      --dependency-types=exec \
      --ignore-src \
      --rosdistro "${ROS_DISTRO}" && \
    rm -rf /tmp/src
