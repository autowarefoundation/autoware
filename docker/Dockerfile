ARG BASE_IMAGE

# hadolint ignore=DL3006
FROM $BASE_IMAGE AS base
SHELL ["/bin/bash", "-o", "pipefail", "-c"]
ARG ROS_DISTRO
ARG SETUP_ARGS

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
  ./setup-dev-env.sh -y --module base ${SETUP_ARGS} --no-cuda-drivers --runtime openadkit \
  && pip uninstall -y ansible ansible-core \
  && apt-get autoremove -y && rm -rf "$HOME"/.cache \
  && echo "source /opt/ros/${ROS_DISTRO}/setup.bash" > /etc/bash.bashrc

# Create entrypoint
COPY docker/etc/ros_entrypoint.sh /ros_entrypoint.sh
RUN chmod +x /ros_entrypoint.sh
ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["/bin/bash"]

# hadolint ignore=DL3006
FROM $BASE_IMAGE AS rosdep-depend
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

COPY src/universe/external /autoware/src/universe/external
COPY src/universe/autoware.universe/common /autoware/src/universe/autoware.universe/common
RUN rosdep keys --ignore-src --from-paths src \
    | xargs rosdep resolve --rosdistro ${ROS_DISTRO} \
    | grep -v '^#' \
    | sed 's/ \+/\n/g'\
    | sort \
    > /rosdep-universe-common-depend-packages.txt \
    && cat /rosdep-universe-common-depend-packages.txt

FROM rosdep-depend AS rosdep-universe-sensing-perception-depend
SHELL ["/bin/bash", "-o", "pipefail", "-c"]
ARG ROS_DISTRO

COPY src/universe/autoware.universe/perception /autoware/src/universe/autoware.universe/perception
COPY src/universe/autoware.universe/sensing /autoware/src/universe/autoware.universe/sensing
RUN rosdep keys --ignore-src --from-paths src \
    | xargs rosdep resolve --rosdistro ${ROS_DISTRO} \
    | grep -v '^#' \
    | sed 's/ \+/\n/g'\
    | sort \
    > /rosdep-universe-sensing-perception-depend-packages.txt \
    && cat /rosdep-universe-sensing-perception-depend-packages.txt
RUN rosdep keys --dependency-types=exec --ignore-src --from-paths src \
    | xargs rosdep resolve --rosdistro ${ROS_DISTRO} \
    | grep -v '^#' \
    | sed 's/ \+/\n/g'\
    | sort \
    > /rosdep-universe-sensing-perception-exec-depend-packages.txt \
    && cat /rosdep-universe-sensing-perception-exec-depend-packages.txt

FROM rosdep-depend AS rosdep-universe-localization-mapping-depend
SHELL ["/bin/bash", "-o", "pipefail", "-c"]
ARG ROS_DISTRO

COPY src/universe/autoware.universe/localization /autoware/src/universe/autoware.universe/localization
COPY src/universe/autoware.universe/map /autoware/src/universe/autoware.universe/map
RUN rosdep keys --ignore-src --from-paths src \
    | xargs rosdep resolve --rosdistro ${ROS_DISTRO} \
    | grep -v '^#' \
    | sed 's/ \+/\n/g'\
    | sort \
    > /rosdep-universe-localization-mapping-depend-packages.txt \
    && cat /rosdep-universe-localization-mapping-depend-packages.txt
RUN rosdep keys --dependency-types=exec --ignore-src --from-paths src \
    | xargs rosdep resolve --rosdistro ${ROS_DISTRO} \
    | grep -v '^#' \
    | sed 's/ \+/\n/g'\
    | sort \
    > /rosdep-universe-localization-mapping-exec-depend-packages.txt \
    && cat /rosdep-universe-localization-mapping-exec-depend-packages.txt

FROM rosdep-depend AS rosdep-universe-planning-control-depend
SHELL ["/bin/bash", "-o", "pipefail", "-c"]
ARG ROS_DISTRO

COPY src/universe/autoware.universe/control /autoware/src/universe/autoware.universe/control
COPY src/universe/autoware.universe/planning /autoware/src/universe/autoware.universe/planning
# TODO(youtalk): Remove COPYs when https://github.com/autowarefoundation/autoware.universe/issues/8805 is resolved
COPY src/universe/autoware.universe/map/map_loader /autoware/src/universe/autoware.universe/map/map_loader
COPY src/universe/autoware.universe/map/autoware_map_projection_loader /autoware/src/universe/autoware.universe/map/autoware_map_projection_loader
COPY src/universe/autoware.universe/sensing/autoware_pcl_extensions /autoware/src/universe/autoware.universe/sensing/autoware_pcl_extensions
COPY src/universe/autoware.universe/sensing/autoware_pointcloud_preprocessor /autoware/src/universe/autoware.universe/sensing/autoware_pointcloud_preprocessor
RUN rosdep keys --ignore-src --from-paths src \
    | xargs rosdep resolve --rosdistro ${ROS_DISTRO} \
    | grep -v '^#' \
    | sed 's/ \+/\n/g'\
    | sort \
    > /rosdep-universe-planning-control-depend-packages.txt \
    && cat /rosdep-universe-planning-control-depend-packages.txt
RUN rosdep keys --dependency-types=exec --ignore-src --from-paths src \
    | xargs rosdep resolve --rosdistro ${ROS_DISTRO} \
    | grep -v '^#' \
    | sed 's/ \+/\n/g'\
    | sort \
    > /rosdep-universe-planning-control-exec-depend-packages.txt \
    && cat /rosdep-universe-planning-control-exec-depend-packages.txt

FROM rosdep-depend AS rosdep-universe-vehicle-system-depend
SHELL ["/bin/bash", "-o", "pipefail", "-c"]
ARG ROS_DISTRO

COPY src/universe/autoware.universe/vehicle /autoware/src/universe/autoware.universe/vehicle
COPY src/universe/autoware.universe/system /autoware/src/universe/autoware.universe/system
COPY src/universe/autoware.universe/map/autoware_map_height_fitter /autoware/src/universe/autoware.universe/map/autoware_map_height_fitter
COPY src/universe/autoware.universe/localization/autoware_pose2twist /autoware/src/universe/autoware.universe/localization/autoware_pose2twist
RUN rosdep keys --ignore-src --from-paths src \
    | xargs rosdep resolve --rosdistro ${ROS_DISTRO} \
    | grep -v '^#' \
    | sed 's/ \+/\n/g'\
    | sort \
    > /rosdep-universe-vehicle-system-depend-packages.txt \
    && cat /rosdep-universe-vehicle-system-depend-packages.txt
RUN rosdep keys --dependency-types=exec --ignore-src --from-paths src \
    | xargs rosdep resolve --rosdistro ${ROS_DISTRO} \
    | grep -v '^#' \
    | sed 's/ \+/\n/g'\
    | sort \
    > /rosdep-universe-vehicle-system-exec-depend-packages.txt \
    && cat /rosdep-universe-vehicle-system-exec-depend-packages.txt

FROM rosdep-depend AS rosdep-universe-depend
SHELL ["/bin/bash", "-o", "pipefail", "-c"]
ARG ROS_DISTRO

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

FROM base AS core-devel
SHELL ["/bin/bash", "-o", "pipefail", "-c"]
ARG ROS_DISTRO
ARG SETUP_ARGS
ENV CCACHE_DIR="/root/.ccache"

# Set up development environment and tools
RUN --mount=type=ssh \
  --mount=type=cache,target=/var/cache/apt,sharing=locked \
  ./setup-dev-env.sh -y --module all ${SETUP_ARGS} --no-cuda-drivers openadkit \
  && ./setup-dev-env.sh -y --module dev-tools openadkit \
  && pip uninstall -y ansible ansible-core \
  && apt-get autoremove -y && rm -rf "$HOME"/.cache

# Install rosdep dependencies
COPY --from=rosdep-depend /rosdep-core-depend-packages.txt /tmp/rosdep-core-depend-packages.txt
# hadolint ignore=SC2002
RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
  apt-get update \
  && cat /tmp/rosdep-core-depend-packages.txt | xargs apt-get install -y --no-install-recommends \
  && apt-get autoremove -y && rm -rf "$HOME"/.cache

RUN --mount=type=cache,target=${CCACHE_DIR} \
  --mount=type=bind,from=rosdep-depend,source=/autoware/src/core,target=/autoware/src/core \
  source /opt/ros/"$ROS_DISTRO"/setup.bash \
  && du -sh ${CCACHE_DIR} && ccache -s \
  && colcon build --cmake-args \
    " -Wno-dev" \
    " --no-warn-unused-cli" \
    --merge-install \
    --install-base /opt/autoware \
    --mixin release compile-commands ccache \
  && du -sh ${CCACHE_DIR} && ccache -s \
  && rm -rf /autoware/build /autoware/log

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["/bin/bash"]

FROM core-devel AS universe-common-devel
SHELL ["/bin/bash", "-o", "pipefail", "-c"]
ARG ROS_DISTRO
ENV CCACHE_DIR="/root/.ccache"

# Install rosdep dependencies
COPY --from=rosdep-depend /rosdep-universe-common-depend-packages.txt /tmp/rosdep-universe-common-depend-packages.txt
# hadolint ignore=SC2002
RUN --mount=type=ssh \
  apt-get update \
  && cat /tmp/rosdep-universe-common-depend-packages.txt | xargs apt-get install -y --no-install-recommends \
  && apt-get autoremove -y && apt-get clean -y && rm -rf /var/lib/apt/lists/* "$HOME"/.cache

# hadolint ignore=SC1091
RUN --mount=type=cache,target=${CCACHE_DIR} \
  --mount=type=bind,from=rosdep-depend,source=/autoware/src/universe/autoware.universe/common,target=/autoware/src/universe/autoware.universe/common \
  --mount=type=bind,from=rosdep-depend,source=/autoware/src/universe/external,target=/autoware/src/universe/external \
  source /opt/ros/"$ROS_DISTRO"/setup.bash \
  && source /opt/autoware/setup.bash \
  && du -sh ${CCACHE_DIR} && ccache -s \
  && colcon build --cmake-args \
    " -Wno-dev" \
    " --no-warn-unused-cli" \
    --merge-install \
    --install-base /opt/autoware \
    --mixin release compile-commands ccache \
  && du -sh ${CCACHE_DIR} && ccache -s \
  && rm -rf /autoware/build /autoware/log

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["/bin/bash"]

FROM universe-common-devel AS universe-sensing-perception-devel
SHELL ["/bin/bash", "-o", "pipefail", "-c"]
ARG ROS_DISTRO
ENV CCACHE_DIR="/root/.ccache"

# Install rosdep dependencies
COPY --from=rosdep-universe-sensing-perception-depend /rosdep-universe-sensing-perception-depend-packages.txt /tmp/rosdep-universe-sensing-perception-depend-packages.txt
# hadolint ignore=SC2002
RUN --mount=type=ssh \
  apt-get update \
  && cat /tmp/rosdep-universe-sensing-perception-depend-packages.txt | xargs apt-get install -y --no-install-recommends \
  && apt-get autoremove -y && apt-get clean -y && rm -rf /var/lib/apt/lists/* "$HOME"/.cache

# hadolint ignore=SC1091
RUN --mount=type=cache,target=${CCACHE_DIR} \
  --mount=type=bind,from=rosdep-universe-sensing-perception-depend,source=/autoware/src/universe/autoware.universe/perception,target=/autoware/src/universe/autoware.universe/perception \
  --mount=type=bind,from=rosdep-universe-sensing-perception-depend,source=/autoware/src/universe/autoware.universe/sensing,target=/autoware/src/universe/autoware.universe/sensing \
  source /opt/ros/"$ROS_DISTRO"/setup.bash \
  && source /opt/autoware/setup.bash \
  && du -sh ${CCACHE_DIR} && ccache -s \
  && colcon build --cmake-args \
    " -Wno-dev" \
    " --no-warn-unused-cli" \
    --merge-install \
    --install-base /opt/autoware \
    --mixin release compile-commands ccache \
  && du -sh ${CCACHE_DIR} && ccache -s \
  && rm -rf /autoware/build /autoware/log

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["/bin/bash"]

FROM universe-common-devel AS universe-localization-mapping-devel
SHELL ["/bin/bash", "-o", "pipefail", "-c"]
ARG ROS_DISTRO
ENV CCACHE_DIR="/root/.ccache"

# Install rosdep dependencies
COPY --from=rosdep-universe-localization-mapping-depend /rosdep-universe-localization-mapping-depend-packages.txt /tmp/rosdep-universe-localization-mapping-depend-packages.txt
# hadolint ignore=SC2002
RUN --mount=type=ssh \
  apt-get update \
  && cat /tmp/rosdep-universe-localization-mapping-depend-packages.txt | xargs apt-get install -y --no-install-recommends \
  && apt-get autoremove -y && apt-get clean -y && rm -rf /var/lib/apt/lists/* "$HOME"/.cache

# hadolint ignore=SC1091
RUN --mount=type=cache,target=${CCACHE_DIR} \
  --mount=type=bind,from=rosdep-universe-localization-mapping-depend,source=/autoware/src/universe/autoware.universe/localization,target=/autoware/src/universe/autoware.universe/localization \
  --mount=type=bind,from=rosdep-universe-localization-mapping-depend,source=/autoware/src/universe/autoware.universe/map,target=/autoware/src/universe/autoware.universe/map \
  source /opt/ros/"$ROS_DISTRO"/setup.bash \
  && source /opt/autoware/setup.bash \
  && du -sh ${CCACHE_DIR} && ccache -s \
  && colcon build --cmake-args \
    " -Wno-dev" \
    " --no-warn-unused-cli" \
    --merge-install \
    --install-base /opt/autoware \
    --mixin release compile-commands ccache \
  && du -sh ${CCACHE_DIR} && ccache -s \
  && rm -rf /autoware/build /autoware/log

FROM universe-common-devel AS universe-planning-control-devel
SHELL ["/bin/bash", "-o", "pipefail", "-c"]
ARG ROS_DISTRO
ENV CCACHE_DIR="/root/.ccache"

# Install rosdep dependencies
COPY --from=rosdep-universe-planning-control-depend /rosdep-universe-planning-control-depend-packages.txt /tmp/rosdep-universe-planning-control-depend-packages.txt
# hadolint ignore=SC2002
RUN --mount=type=ssh \
  apt-get update \
  && cat /tmp/rosdep-universe-planning-control-depend-packages.txt | xargs apt-get install -y --no-install-recommends \
  && apt-get autoremove -y && apt-get clean -y && rm -rf /var/lib/apt/lists/* "$HOME"/.cache

# hadolint ignore=SC1091
RUN --mount=type=cache,target=${CCACHE_DIR} \
  --mount=type=bind,from=rosdep-universe-planning-control-depend,source=/autoware/src/universe/autoware.universe/control,target=/autoware/src/universe/autoware.universe/control \
  --mount=type=bind,from=rosdep-universe-planning-control-depend,source=/autoware/src/universe/autoware.universe/planning,target=/autoware/src/universe/autoware.universe/planning \
  # TODO(youtalk): Remove --mount options when https://github.com/autowarefoundation/autoware.universe/issues/8805 is resolved
  --mount=type=bind,from=rosdep-universe-planning-control-depend,source=/autoware/src/universe/autoware.universe/map/map_loader,target=/autoware/src/universe/autoware.universe/map/map_loader \
  --mount=type=bind,from=rosdep-universe-planning-control-depend,source=/autoware/src/universe/autoware.universe/map/autoware_map_projection_loader,target=/autoware/src/universe/autoware.universe/map/autoware_map_projection_loader \
  --mount=type=bind,from=rosdep-universe-planning-control-depend,source=/autoware/src/universe/autoware.universe/sensing/autoware_pcl_extensions,target=/autoware/src/universe/autoware.universe/sensing/autoware_pcl_extensions \
  --mount=type=bind,from=rosdep-universe-planning-control-depend,source=/autoware/src/universe/autoware.universe/sensing/autoware_pointcloud_preprocessor,target=/autoware/src/universe/autoware.universe/sensing/autoware_pointcloud_preprocessor \
  source /opt/ros/"$ROS_DISTRO"/setup.bash \
  && source /opt/autoware/setup.bash \
  && du -sh ${CCACHE_DIR} && ccache -s \
  && colcon build --cmake-args \
    " -Wno-dev" \
    " --no-warn-unused-cli" \
    --merge-install \
    --install-base /opt/autoware \
    --mixin release compile-commands ccache \
  && du -sh ${CCACHE_DIR} && ccache -s \
  && rm -rf /autoware/build /autoware/log

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["/bin/bash"]

FROM universe-common-devel AS universe-vehicle-system-devel
SHELL ["/bin/bash", "-o", "pipefail", "-c"]
ARG ROS_DISTRO
ENV CCACHE_DIR="/root/.ccache"

# Install rosdep dependencies
COPY --from=rosdep-universe-vehicle-system-depend /rosdep-universe-vehicle-system-depend-packages.txt /tmp/rosdep-universe-vehicle-system-depend-packages.txt
# hadolint ignore=SC2002
RUN --mount=type=ssh \
  apt-get update \
  && cat /tmp/rosdep-universe-vehicle-system-depend-packages.txt | xargs apt-get install -y --no-install-recommends \
  && apt-get autoremove -y && apt-get clean -y && rm -rf /var/lib/apt/lists/* "$HOME"/.cache

# hadolint ignore=SC1091
RUN --mount=type=cache,target=${CCACHE_DIR} \
  --mount=type=bind,from=rosdep-universe-vehicle-system-depend,source=/autoware/src/universe/autoware.universe/vehicle,target=/autoware/src/universe/autoware.universe/vehicle \
  --mount=type=bind,from=rosdep-universe-vehicle-system-depend,source=/autoware/src/universe/autoware.universe/system,target=/autoware/src/universe/autoware.universe/system \
  --mount=type=bind,from=rosdep-universe-vehicle-system-depend,source=/autoware/src/universe/autoware.universe/map/autoware_map_height_fitter,target=/autoware/src/universe/autoware.universe/map/autoware_map_height_fitter \
  --mount=type=bind,from=rosdep-universe-vehicle-system-depend,source=/autoware/src/universe/autoware.universe/localization/autoware_pose2twist,target=/autoware/src/universe/autoware.universe/localization/autoware_pose2twist \
  source /opt/ros/"$ROS_DISTRO"/setup.bash \
  && source /opt/autoware/setup.bash \
  && du -sh ${CCACHE_DIR} && ccache -s \
  && colcon build --cmake-args \
    " -Wno-dev" \
    " --no-warn-unused-cli" \
    --merge-install \
    --install-base /opt/autoware \
    --mixin release compile-commands ccache \
  && du -sh ${CCACHE_DIR} && ccache -s \
  && rm -rf /autoware/build /autoware/log

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["/bin/bash"]

FROM universe-common-devel AS universe-devel
SHELL ["/bin/bash", "-o", "pipefail", "-c"]
ARG ROS_DISTRO
ENV CCACHE_DIR="/root/.ccache"

# Install rosdep dependencies
COPY --from=rosdep-universe-depend /rosdep-universe-depend-packages.txt /tmp/rosdep-universe-depend-packages.txt
# hadolint ignore=SC2002
RUN --mount=type=ssh \
  apt-get update \
  && cat /tmp/rosdep-universe-depend-packages.txt | xargs apt-get install -y --no-install-recommends \
  && apt-get autoremove -y && apt-get clean -y && rm -rf /var/lib/apt/lists/* "$HOME"/.cache

COPY --from=universe-sensing-perception-devel /opt/autoware /opt/autoware
COPY --from=universe-localization-mapping-devel /opt/autoware /opt/autoware
COPY --from=universe-planning-control-devel /opt/autoware /opt/autoware
COPY --from=universe-vehicle-system-devel /opt/autoware /opt/autoware
# hadolint ignore=SC1091
RUN --mount=type=cache,target=${CCACHE_DIR} \
  --mount=type=bind,from=rosdep-universe-depend,source=/autoware/src/launcher,target=/autoware/src/launcher \
  --mount=type=bind,from=rosdep-universe-depend,source=/autoware/src/param,target=/autoware/src/param \
  --mount=type=bind,from=rosdep-universe-depend,source=/autoware/src/sensor_component,target=/autoware/src/sensor_component \
  --mount=type=bind,from=rosdep-universe-depend,source=/autoware/src/sensor_kit,target=/autoware/src/sensor_kit \
  --mount=type=bind,from=rosdep-universe-depend,source=/autoware/src/universe/autoware.universe/evaluator,target=/autoware/src/universe/autoware.universe/evaluator \
  --mount=type=bind,from=rosdep-universe-depend,source=/autoware/src/universe/autoware.universe/launch,target=/autoware/src/universe/autoware.universe/launch \
  --mount=type=bind,from=rosdep-universe-depend,source=/autoware/src/universe/autoware.universe/simulator,target=/autoware/src/universe/autoware.universe/simulator \
  --mount=type=bind,from=rosdep-universe-depend,source=/autoware/src/universe/autoware.universe/tools,target=/autoware/src/universe/autoware.universe/tools \
  --mount=type=bind,from=rosdep-universe-depend,source=/autoware/src/vehicle,target=/autoware/src/vehicle \
  source /opt/ros/"$ROS_DISTRO"/setup.bash \
  && source /opt/autoware/setup.bash \
  && du -sh ${CCACHE_DIR} && ccache -s \
  && colcon build --cmake-args \
    " -Wno-dev" \
    " --no-warn-unused-cli" \
    --merge-install \
    --install-base /opt/autoware \
    --mixin release compile-commands ccache \
  && du -sh ${CCACHE_DIR} && ccache -s \
  && rm -rf /autoware/build /autoware/log

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["/bin/bash"]

FROM base AS universe-sensing-perception
SHELL ["/bin/bash", "-o", "pipefail", "-c"]
ARG ROS_DISTRO
ARG LIB_DIR
ARG SETUP_ARGS

# Set up runtime environment and artifacts
COPY --from=rosdep-universe-sensing-perception-depend /rosdep-universe-sensing-perception-exec-depend-packages.txt /tmp/rosdep-universe-sensing-perception-exec-depend-packages.txt
# hadolint ignore=SC2002
RUN --mount=type=ssh \
  --mount=type=cache,target=/var/cache/apt,sharing=locked \
  ./setup-dev-env.sh -y --module all ${SETUP_ARGS} --download-artifacts --no-cuda-drivers --runtime openadkit \
  && pip uninstall -y ansible ansible-core \
  && apt-get update \
  && cat /tmp/rosdep-universe-sensing-perception-exec-depend-packages.txt | xargs apt-get install -y --no-install-recommends \
  && apt-get autoremove -y && rm -rf "$HOME"/.cache \
  && find /usr/lib/$LIB_DIR-linux-gnu -name "*.a" -type f -delete \
  && find / -name "*.o" -type f -delete \
  && find / -name "*.h" -type f -delete \
  && find / -name "*.hpp" -type f -delete \
  && rm -rf /autoware/ansible /autoware/ansible-galaxy-requirements.yaml /autoware/setup-dev-env.sh /autoware/*.env \
    /root/.local/pipx /opt/ros/"$ROS_DISTRO"/include /etc/apt/sources.list.d/cuda*.list \
    /etc/apt/sources.list.d/docker.list /etc/apt/sources.list.d/nvidia-docker.list \
    /usr/include /usr/share/doc /usr/lib/gcc /usr/lib/jvm /usr/lib/llvm*

COPY --from=universe-sensing-perception-devel /opt/autoware /opt/autoware

# Copy bash aliases
COPY docker/etc/.bash_aliases /root/.bash_aliases
RUN echo "source /opt/autoware/setup.bash" > /etc/bash.bashrc

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["/bin/bash"]

FROM base AS universe-localization-mapping
SHELL ["/bin/bash", "-o", "pipefail", "-c"]
ARG ROS_DISTRO
ARG LIB_DIR
ARG SETUP_ARGS

# Set up runtime environment and artifacts
COPY --from=rosdep-universe-localization-mapping-depend /rosdep-universe-localization-mapping-exec-depend-packages.txt /tmp/rosdep-universe-localization-mapping-exec-depend-packages.txt
# hadolint ignore=SC2002
RUN --mount=type=ssh \
  --mount=type=cache,target=/var/cache/apt,sharing=locked \
  ./setup-dev-env.sh -y --module all ${SETUP_ARGS} --no-cuda-drivers --runtime openadkit \
  && pip uninstall -y ansible ansible-core \
  && apt-get update \
  && cat /tmp/rosdep-universe-localization-mapping-exec-depend-packages.txt | xargs apt-get install -y --no-install-recommends \
  && apt-get autoremove -y && rm -rf "$HOME"/.cache \
  && find /usr/lib/$LIB_DIR-linux-gnu -name "*.a" -type f -delete \
  && find / -name "*.o" -type f -delete \
  && find / -name "*.h" -type f -delete \
  && find / -name "*.hpp" -type f -delete \
  && rm -rf /autoware/ansible /autoware/ansible-galaxy-requirements.yaml /autoware/setup-dev-env.sh /autoware/*.env \
    /root/.local/pipx /opt/ros/"$ROS_DISTRO"/include /etc/apt/sources.list.d/cuda*.list \
    /etc/apt/sources.list.d/docker.list /etc/apt/sources.list.d/nvidia-docker.list \
    /usr/include /usr/share/doc /usr/lib/gcc /usr/lib/jvm /usr/lib/llvm*

COPY --from=universe-localization-mapping-devel /opt/autoware /opt/autoware

# Copy bash aliases
COPY docker/etc/.bash_aliases /root/.bash_aliases
RUN echo "source /opt/autoware/setup.bash" > /etc/bash.bashrc

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["/bin/bash"]

FROM base AS universe-planning-control
SHELL ["/bin/bash", "-o", "pipefail", "-c"]
ARG ROS_DISTRO
ARG LIB_DIR
ARG SETUP_ARGS

# Set up runtime environment and artifacts
COPY --from=rosdep-universe-planning-control-depend /rosdep-universe-planning-control-exec-depend-packages.txt /tmp/rosdep-universe-planning-control-exec-depend-packages.txt
# hadolint ignore=SC2002
RUN --mount=type=ssh \
  --mount=type=cache,target=/var/cache/apt,sharing=locked \
  ./setup-dev-env.sh -y --module all ${SETUP_ARGS} --no-cuda-drivers --runtime openadkit \
  && pip uninstall -y ansible ansible-core \
  && apt-get update \
  && cat /tmp/rosdep-universe-planning-control-exec-depend-packages.txt | xargs apt-get install -y --no-install-recommends \
  && apt-get autoremove -y && rm -rf "$HOME"/.cache \
  && find /usr/lib/$LIB_DIR-linux-gnu -name "*.a" -type f -delete \
  && find / -name "*.o" -type f -delete \
  && find / -name "*.h" -type f -delete \
  && find / -name "*.hpp" -type f -delete \
  && rm -rf /autoware/ansible /autoware/ansible-galaxy-requirements.yaml /autoware/setup-dev-env.sh /autoware/*.env \
    /root/.local/pipx /opt/ros/"$ROS_DISTRO"/include /etc/apt/sources.list.d/cuda*.list \
    /etc/apt/sources.list.d/docker.list /etc/apt/sources.list.d/nvidia-docker.list \
    /usr/include /usr/share/doc /usr/lib/gcc /usr/lib/jvm /usr/lib/llvm*

COPY --from=universe-planning-control-devel /opt/autoware /opt/autoware

# Copy bash aliases
COPY docker/etc/.bash_aliases /root/.bash_aliases
RUN echo "source /opt/autoware/setup.bash" > /etc/bash.bashrc

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["/bin/bash"]

FROM base AS universe-vehicle-system
SHELL ["/bin/bash", "-o", "pipefail", "-c"]
ARG ROS_DISTRO
ARG LIB_DIR
ARG SETUP_ARGS

# Set up runtime environment and artifacts
COPY --from=rosdep-universe-vehicle-system-depend /rosdep-universe-vehicle-system-exec-depend-packages.txt /tmp/rosdep-universe-vehicle-system-exec-depend-packages.txt
# hadolint ignore=SC2002
RUN --mount=type=ssh \
  --mount=type=cache,target=/var/cache/apt,sharing=locked \
  ./setup-dev-env.sh -y --module all ${SETUP_ARGS} --no-cuda-drivers --runtime openadkit \
  && pip uninstall -y ansible ansible-core \
  && apt-get update \
  && cat /tmp/rosdep-universe-vehicle-system-exec-depend-packages.txt | xargs apt-get install -y --no-install-recommends \
  && apt-get autoremove -y && rm -rf "$HOME"/.cache \
  && find /usr/lib/$LIB_DIR-linux-gnu -name "*.a" -type f -delete \
  && find / -name "*.o" -type f -delete \
  && find / -name "*.h" -type f -delete \
  && find / -name "*.hpp" -type f -delete \
  && rm -rf /autoware/ansible /autoware/ansible-galaxy-requirements.yaml /autoware/setup-dev-env.sh /autoware/*.env \
    /root/.local/pipx /opt/ros/"$ROS_DISTRO"/include /etc/apt/sources.list.d/cuda*.list \
    /etc/apt/sources.list.d/docker.list /etc/apt/sources.list.d/nvidia-docker.list \
    /usr/include /usr/share/doc /usr/lib/gcc /usr/lib/jvm /usr/lib/llvm*

COPY --from=universe-vehicle-system-devel /opt/autoware /opt/autoware

# Copy bash aliases
COPY docker/etc/.bash_aliases /root/.bash_aliases
RUN echo "source /opt/autoware/setup.bash" > /etc/bash.bashrc

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["/bin/bash"]

FROM base AS universe
SHELL ["/bin/bash", "-o", "pipefail", "-c"]
ARG ROS_DISTRO
ARG LIB_DIR
ARG SETUP_ARGS

# Set up runtime environment and artifacts
COPY --from=rosdep-universe-depend /rosdep-exec-depend-packages.txt /tmp/rosdep-exec-depend-packages.txt
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
  && rm -rf /autoware/ansible /autoware/ansible-galaxy-requirements.yaml /autoware/setup-dev-env.sh /autoware/*.env \
    /root/.local/pipx /opt/ros/"$ROS_DISTRO"/include /etc/apt/sources.list.d/cuda*.list \
    /etc/apt/sources.list.d/docker.list /etc/apt/sources.list.d/nvidia-docker.list \
    /usr/include /usr/share/doc /usr/lib/gcc /usr/lib/jvm /usr/lib/llvm*

COPY --from=universe-devel /opt/autoware /opt/autoware

# Copy bash aliases
COPY docker/etc/.bash_aliases /root/.bash_aliases
RUN echo "source /opt/autoware/setup.bash" > /etc/bash.bashrc

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["/bin/bash"]
