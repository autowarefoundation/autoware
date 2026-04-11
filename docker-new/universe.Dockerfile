# syntax=docker/dockerfile:1
# check=skip=InvalidDefaultArgInFrom,UndefinedVar
ARG CORE_DEVEL_IMAGE
ARG CORE_IMAGE

FROM ${CORE_DEVEL_IMAGE} AS universe-dependencies
SHELL ["/bin/bash", "-o", "pipefail", "-c"]

# Install universe ansible roles (repeated in universe-runtime-dependencies below
# because that stage starts from a different base image and cannot share layers)
USER ${USERNAME}
RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,target=/var/lib/apt/lists,sharing=locked \
    --mount=type=cache,target=/home/aw/.cache/pip,uid=1000,gid=1000 \
    --mount=type=cache,target=/home/aw/.cache/pipx,uid=1000,gid=1000 \
    pipx install --include-deps "ansible==10.*" && \
    ansible-playbook autoware.dev_env.autoware_requirements \
      --tags universe \
      --skip-tags core,nvidia,artifacts \
      -e "rosdistro=${ROS_DISTRO}" && \
    sudo rm -rf /opt/acados/.git /opt/acados/examples /opt/acados/docs /opt/acados/test && \
    pipx uninstall ansible
USER root

ENV CMAKE_PREFIX_PATH="/opt/acados${CMAKE_PREFIX_PATH:+:$CMAKE_PREFIX_PATH}"
ENV ACADOS_SOURCE_DIR="/opt/acados"
ENV LD_LIBRARY_PATH="/opt/acados/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"

COPY --parents --chown=${USERNAME}:${USERNAME} src/**/package.xml /tmp/autoware/

RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,target=/var/lib/apt/lists,sharing=locked \
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
      --dependency-types=test

FROM universe-dependencies AS universe-dependencies-cuda

USER ${USERNAME}
RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,target=/var/lib/apt/lists,sharing=locked \
    --mount=type=cache,target=/home/aw/.cache/pipx,uid=1000,gid=1000 \
    pipx install --include-deps "ansible==10.*" && \
    ansible-playbook autoware.dev_env.autoware_requirements \
      --tags nvidia \
      -e "rosdistro=${ROS_DISTRO}" \
      -e install_devel=y \
      -e cuda_install_drivers=false && \
    pipx uninstall ansible
USER root

ENV PATH="/usr/local/cuda/bin${PATH:+:$PATH}"
ENV LD_LIBRARY_PATH="/usr/local/cuda/lib64${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"

FROM universe-dependencies-cuda AS universe-devel-cuda

RUN --mount=type=bind,source=src,target=/tmp/autoware/src \
    --mount=type=cache,target=/home/aw/.ccache,uid=1000,gid=1000,sharing=shared \
    . "/opt/ros/${ROS_DISTRO}/setup.sh" && \
    . /opt/autoware/setup.sh && \
    colcon build \
      --base-paths /tmp/autoware/src \
      --install-base /opt/autoware \
      --cmake-args -DCMAKE_BUILD_TYPE=Release && \
    rm -rf build log

FROM universe-dependencies AS universe-devel

RUN --mount=type=bind,source=src,target=/tmp/autoware/src \
    --mount=type=cache,target=/home/aw/.ccache,uid=1000,gid=1000,sharing=shared \
    CCACHE_READONLY=1 \
    . "/opt/ros/${ROS_DISTRO}/setup.sh" && \
    . /opt/autoware/setup.sh && \
    colcon build \
      --base-paths /tmp/autoware/src \
      --install-base /opt/autoware \
      --cmake-args -DCMAKE_BUILD_TYPE=Release && \
    rm -rf build log

FROM ${CORE_IMAGE} AS universe-runtime-dependencies
SHELL ["/bin/bash", "-o", "pipefail", "-c"]

USER ${USERNAME}
RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,target=/var/lib/apt/lists,sharing=locked \
    --mount=type=cache,target=/home/aw/.cache/pip,uid=1000,gid=1000 \
    --mount=type=cache,target=/home/aw/.cache/pipx,uid=1000,gid=1000 \
    pipx install --include-deps "ansible==10.*" && \
    ansible-playbook autoware.dev_env.autoware_requirements \
      --tags universe \
      --skip-tags core,nvidia,artifacts \
      -e "rosdistro=${ROS_DISTRO}" && \
    sudo rm -rf /opt/acados/.git /opt/acados/examples /opt/acados/docs /opt/acados/test && \
    pipx uninstall ansible
USER root

ENV CMAKE_PREFIX_PATH="/opt/acados${CMAKE_PREFIX_PATH:+:$CMAKE_PREFIX_PATH}"
ENV ACADOS_SOURCE_DIR="/opt/acados"
ENV LD_LIBRARY_PATH="/opt/acados/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"

COPY --parents --chown=${USERNAME}:${USERNAME} src/**/package.xml /tmp/

RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,target=/var/lib/apt/lists,sharing=locked \
    apt-get update && \
    . "/opt/ros/${ROS_DISTRO}/setup.sh" && \
    . /opt/autoware/setup.sh && \
    rosdep install -y --from-paths /tmp/src \
      --dependency-types=exec \
      --ignore-src \
      --rosdistro "${ROS_DISTRO}" && \
    rm -rf /tmp/src

FROM universe-runtime-dependencies AS universe

COPY --from=universe-devel /opt/autoware /opt/autoware
RUN find /opt/autoware -name '*.so' -exec strip --strip-unneeded {} +

FROM universe-runtime-dependencies AS universe-cuda

USER ${USERNAME}
RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,target=/var/lib/apt/lists,sharing=locked \
    --mount=type=cache,target=/home/aw/.cache/pipx,uid=1000,gid=1000 \
    pipx install --include-deps "ansible==10.*" && \
    ansible-playbook autoware.dev_env.autoware_requirements \
      --tags nvidia \
      -e "rosdistro=${ROS_DISTRO}" \
      -e install_devel=N \
      -e cuda_install_drivers=false && \
    pipx uninstall ansible
USER root

ENV PATH="/usr/local/cuda/bin${PATH:+:$PATH}"
ENV LD_LIBRARY_PATH="/usr/local/cuda/lib64${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"

COPY --from=universe-devel-cuda /opt/autoware /opt/autoware
RUN find /opt/autoware -name '*.so' -exec strip --strip-unneeded {} +
