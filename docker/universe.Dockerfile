# syntax=docker/dockerfile:1
# check=skip=InvalidDefaultArgInFrom,UndefinedVar
ARG CORE_DEVEL_IMAGE
ARG CORE_IMAGE

FROM ${CORE_DEVEL_IMAGE} AS universe-dependencies
SHELL ["/bin/bash", "-o", "pipefail", "-c"]
ARG ROS_DISTRO

USER ${USERNAME}
RUN --mount=type=cache,id=apt-cache-${ROS_DISTRO},target=/var/cache/apt,sharing=locked \
    --mount=type=cache,id=apt-lists-${ROS_DISTRO},target=/var/lib/apt/lists,sharing=locked \
    --mount=type=cache,id=pip-cache,target=/home/aw/.cache/pip,uid=1000,gid=1000 \
    --mount=type=cache,id=pipx-cache,target=/home/aw/.cache/pipx,uid=1000,gid=1000 \
    pipx install --include-deps "ansible==10.*" && \
    ansible-playbook autoware.dev_env.autoware_requirements \
      --tags acados \
      -e "rosdistro=${ROS_DISTRO}" && \
    sudo rm -rf /opt/acados/.git /opt/acados/examples /opt/acados/docs /opt/acados/test && \
    pipx uninstall ansible
USER root

ENV CMAKE_PREFIX_PATH="/opt/acados${CMAKE_PREFIX_PATH:+:$CMAKE_PREFIX_PATH}"
ENV ACADOS_SOURCE_DIR="/opt/acados"
ENV LD_LIBRARY_PATH="/opt/acados/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"

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

FROM universe-dependencies AS universe-devel
ARG ROS_DISTRO

RUN --mount=type=bind,source=src,target=/tmp/autoware/src \
    --mount=type=cache,id=ccache-${ROS_DISTRO},target=/home/aw/.ccache,uid=1000,gid=1000,sharing=shared \
    CCACHE_READONLY=1 \
    . "/opt/ros/${ROS_DISTRO}/setup.sh" && \
    . /opt/autoware/setup.sh && \
    colcon build \
      --base-paths /tmp/autoware/src \
      --install-base /opt/autoware \
      --cmake-args -DCMAKE_BUILD_TYPE=Release && \
    rm -rf build log

FROM ${CORE_IMAGE} AS universe
SHELL ["/bin/bash", "-o", "pipefail", "-c"]
ARG ROS_DISTRO

COPY --parents --chown=${USERNAME}:${USERNAME} src/**/package.xml /tmp/

RUN --mount=type=cache,id=apt-cache-${ROS_DISTRO},target=/var/cache/apt,sharing=locked \
    --mount=type=cache,id=apt-lists-${ROS_DISTRO},target=/var/lib/apt/lists,sharing=locked \
    apt-get update && \
    . "/opt/ros/${ROS_DISTRO}/setup.sh" && \
    . /opt/autoware/setup.sh && \
    rosdep install -y --from-paths /tmp/src \
      --dependency-types=exec \
      --ignore-src \
      --rosdistro "${ROS_DISTRO}" && \
    rm -rf /tmp/src

COPY --from=universe-devel /opt/acados /opt/acados
COPY --from=universe-devel /opt/autoware /opt/autoware
RUN find /opt/autoware -name '*.so' -exec strip --strip-unneeded {} +

ENV CMAKE_PREFIX_PATH="/opt/acados${CMAKE_PREFIX_PATH:+:$CMAKE_PREFIX_PATH}"
ENV ACADOS_SOURCE_DIR="/opt/acados"
ENV LD_LIBRARY_PATH="/opt/acados/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
