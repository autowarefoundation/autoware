# check=skip=InvalidDefaultArgInFrom,UndefinedVar
ARG CORE_DEVEL_IMAGE
ARG CORE_IMAGE

FROM ${CORE_DEVEL_IMAGE} AS universe-dependencies
SHELL ["/bin/bash", "-o", "pipefail", "-c"]

COPY --chown=${USERNAME}:${USERNAME} ansible-galaxy-requirements.yaml /tmp/ansible/
COPY --chown=${USERNAME}:${USERNAME} ansible/ /tmp/ansible/ansible/

WORKDIR /tmp/ansible
RUN ansible-galaxy collection install -f -r ansible-galaxy-requirements.yaml && \
    ansible-playbook autoware.dev_env.autoware_requirements \
      --tags universe \
      --skip-tags core,nvidia,artifacts \
      -e "rosdistro=${ROS_DISTRO}"
WORKDIR /home/${USERNAME}
RUN rm -rf /tmp/ansible

ENV CMAKE_PREFIX_PATH="/opt/acados${CMAKE_PREFIX_PATH:+:$CMAKE_PREFIX_PATH}"
ENV ACADOS_SOURCE_DIR="/opt/acados"
ENV LD_LIBRARY_PATH="/opt/acados/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"

COPY --chown=${USERNAME}:${USERNAME} src/ /tmp/autoware/src/

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

COPY --chown=${USERNAME}:${USERNAME} ansible-galaxy-requirements.yaml /tmp/ansible/
COPY --chown=${USERNAME}:${USERNAME} ansible/ /tmp/ansible/ansible/

WORKDIR /tmp/ansible
RUN ansible-galaxy collection install -f -r ansible-galaxy-requirements.yaml && \
    ansible-playbook autoware.dev_env.autoware_requirements \
      --tags nvidia \
      -e "rosdistro=${ROS_DISTRO}" \
      -e install_devel=y \
      -e cuda_install_drivers=false
WORKDIR /home/${USERNAME}
RUN rm -rf /tmp/ansible

ENV PATH="/usr/local/cuda/bin${PATH:+:$PATH}"
ENV LD_LIBRARY_PATH="/usr/local/cuda/lib64${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"

FROM universe-dependencies-cuda AS universe-devel-cuda

RUN --mount=type=cache,target=/home/aw/.ccache,uid=1000,gid=1000 \
    . "/opt/ros/${ROS_DISTRO}/setup.sh" && \
    . /opt/autoware/setup.sh && \
    colcon build \
      --base-paths /tmp/autoware/src \
      --install-base /opt/autoware \
      --cmake-args -DCMAKE_BUILD_TYPE=Release && \
    rm -rf /tmp/autoware

FROM universe-dependencies AS universe-devel

RUN --mount=type=cache,target=/home/aw/.ccache,uid=1000,gid=1000 \
    . "/opt/ros/${ROS_DISTRO}/setup.sh" && \
    . /opt/autoware/setup.sh && \
    colcon build \
      --base-paths /tmp/autoware/src \
      --install-base /opt/autoware \
      --cmake-args -DCMAKE_BUILD_TYPE=Release && \
    rm -rf /tmp/autoware

FROM ${CORE_IMAGE} AS universe-runtime-dependencies
SHELL ["/bin/bash", "-o", "pipefail", "-c"]

COPY --chown=${USERNAME}:${USERNAME} ansible-galaxy-requirements.yaml /tmp/ansible/
COPY --chown=${USERNAME}:${USERNAME} ansible/ /tmp/ansible/ansible/

WORKDIR /tmp/ansible
RUN ansible-galaxy collection install -f -r ansible-galaxy-requirements.yaml && \
    ansible-playbook autoware.dev_env.autoware_requirements \
      --tags universe \
      --skip-tags core,nvidia,artifacts \
      -e "rosdistro=${ROS_DISTRO}"
WORKDIR /home/${USERNAME}
RUN rm -rf /tmp/ansible

ENV CMAKE_PREFIX_PATH="/opt/acados${CMAKE_PREFIX_PATH:+:$CMAKE_PREFIX_PATH}"
ENV ACADOS_SOURCE_DIR="/opt/acados"
ENV LD_LIBRARY_PATH="/opt/acados/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"

COPY --chown=${USERNAME}:${USERNAME} src/ /tmp/src/

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

COPY --chown=${USERNAME}:${USERNAME} ansible-galaxy-requirements.yaml /tmp/ansible/
COPY --chown=${USERNAME}:${USERNAME} ansible/ /tmp/ansible/ansible/

WORKDIR /tmp/ansible
RUN ansible-galaxy collection install -f -r ansible-galaxy-requirements.yaml && \
    ansible-playbook autoware.dev_env.autoware_requirements \
      --tags nvidia \
      -e "rosdistro=${ROS_DISTRO}" \
      -e install_devel=N \
      -e cuda_install_drivers=false
WORKDIR /home/${USERNAME}
RUN rm -rf /tmp/ansible

ENV PATH="/usr/local/cuda/bin${PATH:+:$PATH}"
ENV LD_LIBRARY_PATH="/usr/local/cuda/lib64${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"

COPY --from=universe-devel-cuda /opt/autoware /opt/autoware
RUN find /opt/autoware -name '*.so' -exec strip --strip-unneeded {} +
