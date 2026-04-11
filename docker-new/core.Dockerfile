# syntax=docker/dockerfile:1
# check=skip=InvalidDefaultArgInFrom
ARG BASE_IMAGE

FROM ${BASE_IMAGE} AS core-dependencies
SHELL ["/bin/bash", "-o", "pipefail", "-c"]

USER ${USERNAME}
RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,target=/var/lib/apt/lists,sharing=locked \
    --mount=type=cache,target=/home/aw/.cache/pip,uid=1000,gid=1000 \
    --mount=type=cache,target=/home/aw/.cache/pipx,uid=1000,gid=1000 \
    pipx install --include-deps "ansible==10.*" && \
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

RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,target=/var/lib/apt/lists,sharing=locked \
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
    --mount=type=cache,target=/home/aw/.ccache,uid=1000,gid=1000 \
    rm -rf /tmp/autoware/src/core/autoware_core \
           /tmp/autoware/src/core/autoware_rviz_plugins && \
    . "/opt/ros/${ROS_DISTRO}/setup.sh" && \
    colcon build \
      --base-paths /tmp/autoware/src/core \
      --install-base /opt/autoware \
      --cmake-args -DCMAKE_BUILD_TYPE=Release && \
    rm -rf build log

FROM core-dependencies AS core-devel

COPY --parents --chown=${USERNAME}:${USERNAME} \
    src/core/autoware_core/**/package.xml \
    src/core/autoware_rviz_plugins/**/package.xml \
    /tmp/autoware/

RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,target=/var/lib/apt/lists,sharing=locked \
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
    --mount=type=cache,target=/home/aw/.ccache,uid=1000,gid=1000 \
    . "/opt/ros/${ROS_DISTRO}/setup.sh" && \
    . /opt/autoware/setup.sh && \
    colcon build \
      --base-paths /tmp/autoware/src/core/autoware_core \
                   /tmp/autoware/src/core/autoware_rviz_plugins \
      --install-base /opt/autoware \
      --cmake-args -DCMAKE_BUILD_TYPE=Release && \
    rm -rf build log

FROM ${BASE_IMAGE} AS core
ENV AUTOWARE_RUNTIME=1

COPY --from=core-devel /opt/autoware /opt/autoware
RUN find /opt/autoware -name '*.so' -exec strip --strip-unneeded {} +

COPY --parents src/core/**/package.xml /tmp/

RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,target=/var/lib/apt/lists,sharing=locked \
    apt-get update && \
    . "/opt/ros/${ROS_DISTRO}/setup.sh" && \
    sudo apt-get install -y "ros-${ROS_DISTRO}-topic-tools" && \
    rosdep install -y --from-paths /tmp/src/core \
      --dependency-types=exec \
      --ignore-src \
      --rosdistro "${ROS_DISTRO}" && \
    rm -rf /tmp/src
