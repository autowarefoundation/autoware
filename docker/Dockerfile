# hadolint global ignore=DL3006,DL3008,DL3013
ARG BASE_IMAGE

FROM $BASE_IMAGE as base
SHELL ["/bin/bash", "-o", "pipefail", "-c"]
ARG ROS_DISTRO

# Install apt packages and add GitHub to known hosts for private repositories
RUN rm -f /etc/apt/apt.conf.d/docker-clean \
  && echo 'Binary::apt::APT::Keep-Downloaded-Packages "true";' >/etc/apt/apt.conf.d/keep-cache
RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
  apt-get update && DEBIAN_FRONTEND=noninteractive apt-get -y install --no-install-recommends \
  gosu \
  ssh \
  && apt-get autoremove -y && rm -rf "$HOME"/.cache \
  && mkdir -p ~/.ssh \
  && ssh-keyscan github.com >> ~/.ssh/known_hosts

# Copy files
COPY setup-dev-env.sh ansible-galaxy-requirements.yaml amd64.env arm64.env /autoware/
COPY ansible/ /autoware/ansible/
WORKDIR /autoware

# Set up base environment
RUN --mount=type=ssh \
  --mount=type=cache,target=/var/cache/apt,sharing=locked \
  ./setup-dev-env.sh -y --module base --runtime openadkit \
  && pip uninstall -y ansible ansible-core \
  && apt-get autoremove -y && rm -rf "$HOME"/.cache \
  && echo "source /opt/ros/${ROS_DISTRO}/setup.bash" > /etc/bash.bashrc

# Create entrypoint
CMD ["/bin/bash"]

FROM $BASE_IMAGE as rosdep-depend
SHELL ["/bin/bash", "-o", "pipefail", "-c"]
ARG ROS_DISTRO

COPY setup-dev-env.sh ansible-galaxy-requirements.yaml amd64.env arm64.env /autoware/
COPY ansible/ /autoware/ansible/
WORKDIR /autoware

RUN rm -f /etc/apt/apt.conf.d/docker-clean \
  && echo 'Binary::apt::APT::Keep-Downloaded-Packages "true";' >/etc/apt/apt.conf.d/keep-cache
RUN --mount=type=ssh \
  --mount=type=cache,target=/var/cache/apt,sharing=locked \
  ./setup-dev-env.sh -y rosdep \
  && pip uninstall -y ansible ansible-core \
  && apt-get autoremove -y && rm -rf "$HOME"/.cache

# Generate install package lists
COPY src/core /autoware/src/core
RUN rosdep update && rosdep keys --ignore-src --from-paths src \
    | xargs rosdep resolve --rosdistro ${ROS_DISTRO} \
    | grep -v '^#' \
    | sed 's/ \+/\n/g'\
    | sort \
    > /rosdep-core-depend-packages.txt \
    && cat /rosdep-core-depend-packages.txt
COPY src/launcher /autoware/src/launcher
COPY src/param /autoware/src/param
COPY src/sensor_component /autoware/src/sensor_component
COPY src/sensor_kit /autoware/src/sensor_kit
COPY src/universe /autoware/src/universe
COPY src/vehicle /autoware/src/vehicle
RUN rosdep keys --ignore-src --from-paths src \
    | xargs rosdep resolve --rosdistro ${ROS_DISTRO} \
    | grep -v '^#' \
    | sed 's/ \+/\n/g'\
    | sort \
    > /rosdep-universe-depend-packages.txt \
    && cat /rosdep-universe-depend-packages.txt
RUN rosdep keys --dependency-types=exec --ignore-src --from-paths src \
    | xargs rosdep resolve --rosdistro ${ROS_DISTRO} \
    | grep -v '^#' \
    | sed 's/ \+/\n/g'\
    | sort \
    > /rosdep-exec-depend-packages.txt \
    && cat /rosdep-exec-depend-packages.txt

FROM base as autoware-core
SHELL ["/bin/bash", "-o", "pipefail", "-c"]
ARG ROS_DISTRO
ARG SETUP_ARGS
ENV CCACHE_DIR="/root/.ccache"

# cspell: ignore libcu libnv
# Set up development environment
RUN --mount=type=ssh \
  --mount=type=cache,target=/var/cache/apt,sharing=locked \
  ./setup-dev-env.sh -y --module all ${SETUP_ARGS} --no-cuda-drivers openadkit \
  && pip uninstall -y ansible ansible-core \
  && apt-get autoremove -y && rm -rf "$HOME"/.cache

# Install rosdep dependencies
COPY --from=rosdep-depend /rosdep-core-depend-packages.txt /tmp/rosdep-core-depend-packages.txt
# hadolint ignore=SC2002
RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
  apt-get update \
  && cat /tmp/rosdep-core-depend-packages.txt | xargs apt-get install -y --no-install-recommends \
  && apt-get autoremove -y && rm -rf "$HOME"/.cache

RUN --mount=type=bind,from=rosdep-depend,source=/autoware/src/core,target=/autoware/src/core \
  --mount=type=cache,target=${CCACHE_DIR} \
  source /opt/ros/"$ROS_DISTRO"/setup.bash \
  && du -sh ${CCACHE_DIR} && ccache -s \
  && colcon build --cmake-args \
    " -Wno-dev" \
    " --no-warn-unused-cli" \
    --mixin release compile-commands ccache \
  && du -sh ${CCACHE_DIR} && ccache -s

FROM autoware-core as autoware-universe
SHELL ["/bin/bash", "-o", "pipefail", "-c"]
ARG ROS_DISTRO
ARG SETUP_ARGS
ENV CCACHE_DIR="/root/.ccache"

# Install rosdep dependencies
COPY --from=rosdep-depend /rosdep-universe-depend-packages.txt /tmp/rosdep-universe-depend-packages.txt
# hadolint ignore=SC2002
RUN --mount=type=ssh \
  apt-get update \
  && cat /tmp/rosdep-universe-depend-packages.txt | xargs apt-get install -y --no-install-recommends \
  && apt-get autoremove -y && apt-get clean -y && rm -rf /var/lib/apt/lists/* "$HOME"/.cache

# Build Autoware
RUN --mount=type=bind,from=rosdep-depend,source=/autoware/src,target=/autoware/src \
  --mount=type=cache,target=${CCACHE_DIR} \
  source /opt/ros/"$ROS_DISTRO"/setup.bash \
  && du -sh ${CCACHE_DIR} && ccache -s \
  && colcon build --cmake-args \
    " -Wno-dev" \
    " --no-warn-unused-cli" \
    --mixin release compile-commands ccache \
  && du -sh ${CCACHE_DIR} && ccache -s

CMD ["/bin/bash"]

FROM autoware-universe as devel
SHELL ["/bin/bash", "-o", "pipefail", "-c"]

# Install development tools and artifacts
RUN --mount=type=ssh \
  --mount=type=cache,target=/var/cache/apt,sharing=locked \
  ./setup-dev-env.sh -y --module dev-tools openadkit \
  && pip uninstall -y ansible ansible-core \
  && apt-get autoremove -y && rm -rf "$HOME"/.cache

COPY src /autoware/src
# Create entrypoint
COPY docker/etc/ros_entrypoint.sh /ros_entrypoint.sh
RUN chmod +x /ros_entrypoint.sh
ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["/bin/bash"]

FROM base as runtime
SHELL ["/bin/bash", "-o", "pipefail", "-c"]
ARG ROS_DISTRO
ARG LIB_DIR
ARG SETUP_ARGS

# Set up runtime environment and artifacts
COPY --from=rosdep-depend /rosdep-exec-depend-packages.txt /tmp/rosdep-exec-depend-packages.txt
# hadolint ignore=SC2002
RUN --mount=type=ssh \
  --mount=type=cache,target=/var/cache/apt,sharing=locked \
  ./setup-dev-env.sh -y --module all ${SETUP_ARGS} --download-artifacts --no-cuda-drivers --runtime openadkit \
  && pip uninstall -y ansible ansible-core \
  && apt-get update \
  && cat /tmp/rosdep-exec-depend-packages.txt | xargs apt-get install -y --no-install-recommends \
  && apt-get autoremove -y && rm -rf "$HOME"/.cache \
  && find /usr/lib/$LIB_DIR-linux-gnu -name "*.a" -type f -delete \
  && find / -name "*.o" -type f -delete \
  && find / -name "*.h" -type f -delete \
  && find / -name "*.hpp" -type f -delete \
  && rm -rf /autoware/ansible \
    /root/.local/pipx /opt/ros/"$ROS_DISTRO"/include /etc/apt/sources.list.d/cuda*.list \
    /etc/apt/sources.list.d/docker.list /etc/apt/sources.list.d/nvidia-docker.list \
    /usr/include /usr/share/doc /usr/lib/gcc /usr/lib/jvm /usr/lib/llvm*

COPY --from=autoware-universe /autoware/install/ /autoware/install/

# Copy bash aliases
COPY docker/etc/.bash_aliases /root/.bash_aliases
RUN echo "source /autoware/install/setup.bash" > /etc/bash.bashrc

# Create entrypoint
COPY docker/etc/ros_entrypoint.sh /ros_entrypoint.sh
RUN chmod +x /ros_entrypoint.sh
ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
