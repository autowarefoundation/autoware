ARG AUTOWARE_BASE_IMAGE=ghcr.io/autowarefoundation/autoware-base:latest
ARG AUTOWARE_BASE_CUDA_IMAGE=ghcr.io/autowarefoundation/autoware-base:cuda-latest
ARG AUTOWARE_BASE_IMAGE_JAZZY=ghcr.io/autowarefoundation/autoware-base:jazzy-latest
ARG ROS_DISTRO=humble

# hadolint ignore=DL3006
FROM $AUTOWARE_BASE_IMAGE AS rosdep-depend
SHELL ["/bin/bash", "-o", "pipefail", "-c"]

COPY setup-dev-env.sh ansible-galaxy-requirements.yaml amd64.env amd64_jazzy.env arm64.env /autoware/
COPY ansible/ /autoware/ansible/
COPY docker/scripts/cleanup_apt.sh /autoware/cleanup_apt.sh
RUN chmod +x /autoware/cleanup_apt.sh
COPY docker/scripts/resolve_rosdep_keys.sh /autoware/resolve_rosdep_keys.sh
RUN chmod +x /autoware/resolve_rosdep_keys.sh
WORKDIR /autoware

RUN rm -f /etc/apt/apt.conf.d/docker-clean \
  && echo 'Binary::apt::APT::Keep-Downloaded-Packages "true";' >/etc/apt/apt.conf.d/keep-cache \
  && /autoware/cleanup_apt.sh

# Generate install package lists
COPY src/core/autoware_adapi_msgs /autoware/src/core/autoware_adapi_msgs
COPY src/core/autoware_cmake /autoware/src/core/autoware_cmake
COPY src/core/autoware_internal_msgs /autoware/src/core/autoware_internal_msgs
COPY src/core/autoware_lanelet2_extension /autoware/src/core/autoware_lanelet2_extension
COPY src/core/autoware_msgs /autoware/src/core/autoware_msgs
COPY src/core/autoware_utils /autoware/src/core/autoware_utils
RUN rosdep update && /autoware/resolve_rosdep_keys.sh /autoware/src "${ROS_DISTRO}" \
  > /rosdep-core-common-depend-packages.txt \
  && cat /rosdep-core-common-depend-packages.txt
RUN /autoware/resolve_rosdep_keys.sh /autoware/src "${ROS_DISTRO}" --dependency-types=exec \
  > /rosdep-core-common-exec-depend-packages.txt \
  && cat /rosdep-core-common-exec-depend-packages.txt

COPY src/core/autoware_core /autoware/src/core/autoware_core
RUN rosdep update && /autoware/resolve_rosdep_keys.sh /autoware/src "${ROS_DISTRO}" \
  > /rosdep-core-depend-packages.txt \
  && cat /rosdep-core-depend-packages.txt
RUN /autoware/resolve_rosdep_keys.sh /autoware/src "${ROS_DISTRO}" --dependency-types=exec \
  > /rosdep-core-exec-depend-packages.txt \
  && cat /rosdep-core-exec-depend-packages.txt

COPY src/universe/external /autoware/src/universe/external
# trt_batched_nms depends on autoware_tensorrt_common and autoware_cuda_utils, which are not available in universe-common-devel stage
RUN rm -rf /autoware/src/universe/external/trt_batched_nms
COPY src/universe/autoware_universe/common /autoware/src/universe/autoware_universe/common
COPY src/middleware/external /autoware/src/middleware/external
RUN /autoware/resolve_rosdep_keys.sh /autoware/src "${ROS_DISTRO}" \
  > /rosdep-universe-common-depend-packages.txt \
  && cat /rosdep-universe-common-depend-packages.txt

FROM rosdep-depend AS rosdep-universe-visualization-depend
SHELL ["/bin/bash", "-o", "pipefail", "-c"]
ARG ROS_DISTRO

COPY src/universe/autoware_universe/visualization /autoware/src/universe/autoware_universe/visualization
COPY src/core/autoware_rviz_plugins /autoware/src/core/autoware_rviz_plugins

RUN /autoware/resolve_rosdep_keys.sh /autoware/src "${ROS_DISTRO}" \
  > /rosdep-universe-visualization-depend-packages.txt \
  && cat /rosdep-universe-visualization-depend-packages.txt
RUN /autoware/resolve_rosdep_keys.sh /autoware/src "${ROS_DISTRO}" --dependency-types=exec \
  > /rosdep-universe-visualization-exec-depend-packages.txt \
  && cat /rosdep-universe-visualization-exec-depend-packages.txt

FROM rosdep-depend AS rosdep-universe-sensing-perception-depend
SHELL ["/bin/bash", "-o", "pipefail", "-c"]
ARG ROS_DISTRO

COPY src/launcher/autoware_launch/tier4_universe_launch/tier4_perception_launch /autoware/src/launcher/autoware_launch/tier4_universe_launch/tier4_perception_launch
COPY src/launcher/autoware_launch/tier4_universe_launch/tier4_sensing_launch /autoware/src/launcher/autoware_launch/tier4_universe_launch/tier4_sensing_launch
COPY src/universe/autoware_universe/perception /autoware/src/universe/autoware_universe/perception
COPY src/universe/autoware_universe/sensing /autoware/src/universe/autoware_universe/sensing
COPY src/universe/autoware_universe/evaluator/autoware_perception_online_evaluator /autoware/src/universe/autoware_universe/evaluator/autoware_perception_online_evaluator
COPY src/sensor_component /autoware/src/sensor_component
RUN /autoware/resolve_rosdep_keys.sh /autoware/src "${ROS_DISTRO}" \
  > /rosdep-universe-sensing-perception-depend-packages.txt \
  && cat /rosdep-universe-sensing-perception-depend-packages.txt
RUN /autoware/resolve_rosdep_keys.sh /autoware/src "${ROS_DISTRO}" --dependency-types=exec \
  > /rosdep-universe-sensing-perception-exec-depend-packages.txt \
  && cat /rosdep-universe-sensing-perception-exec-depend-packages.txt

FROM rosdep-depend AS rosdep-universe-localization-mapping-depend
SHELL ["/bin/bash", "-o", "pipefail", "-c"]
ARG ROS_DISTRO

COPY src/launcher/autoware_launch/tier4_universe_launch/tier4_localization_launch /autoware/src/launcher/autoware_launch/tier4_universe_launch/tier4_localization_launch
COPY src/launcher/autoware_launch/tier4_universe_launch/tier4_map_launch /autoware/src/launcher/autoware_launch/tier4_universe_launch/tier4_map_launch
COPY src/universe/autoware_universe/localization /autoware/src/universe/autoware_universe/localization
COPY src/universe/autoware_universe/map /autoware/src/universe/autoware_universe/map
# TODO(youtalk): Remove COPYs when https://github.com/autowarefoundation/autoware_universe/issues/10282 is resolved
COPY src/universe/autoware_universe/sensing/autoware_pcl_extensions /autoware/src/universe/autoware_universe/sensing/autoware_pcl_extensions
COPY src/universe/autoware_universe/sensing/autoware_pointcloud_preprocessor /autoware/src/universe/autoware_universe/sensing/autoware_pointcloud_preprocessor
COPY src/universe/autoware_universe/system/autoware_default_adapi_helpers/autoware_automatic_pose_initializer /autoware/src/universe/autoware_universe/system/autoware_default_adapi_helpers/autoware_automatic_pose_initializer
RUN /autoware/resolve_rosdep_keys.sh /autoware/src "${ROS_DISTRO}" \
  > /rosdep-universe-localization-mapping-depend-packages.txt \
  && cat /rosdep-universe-localization-mapping-depend-packages.txt
RUN /autoware/resolve_rosdep_keys.sh /autoware/src "${ROS_DISTRO}" --dependency-types=exec \
  > /rosdep-universe-localization-mapping-exec-depend-packages.txt \
  && cat /rosdep-universe-localization-mapping-exec-depend-packages.txt

FROM rosdep-depend AS rosdep-universe-planning-control-depend
SHELL ["/bin/bash", "-o", "pipefail", "-c"]
ARG ROS_DISTRO

COPY src/launcher/autoware_launch/tier4_universe_launch/tier4_control_launch /autoware/src/launcher/autoware_launch/tier4_universe_launch/tier4_control_launch
COPY src/launcher/autoware_launch/tier4_universe_launch/tier4_planning_launch /autoware/src/launcher/autoware_launch/tier4_universe_launch/tier4_planning_launch
COPY src/universe/autoware_universe/control /autoware/src/universe/autoware_universe/control
COPY src/universe/autoware_universe/planning /autoware/src/universe/autoware_universe/planning
# TODO(youtalk): Remove COPYs when https://github.com/autowarefoundation/autoware_universe/issues/8805 is resolved
COPY src/universe/autoware_universe/evaluator/autoware_control_evaluator /autoware/src/universe/autoware_universe/evaluator/autoware_control_evaluator
COPY src/universe/autoware_universe/evaluator/autoware_planning_evaluator /autoware/src/universe/autoware_universe/evaluator/autoware_planning_evaluator
COPY src/universe/autoware_universe/sensing/autoware_pcl_extensions /autoware/src/universe/autoware_universe/sensing/autoware_pcl_extensions
COPY src/universe/autoware_universe/sensing/autoware_pointcloud_preprocessor /autoware/src/universe/autoware_universe/sensing/autoware_pointcloud_preprocessor
COPY src/universe/autoware_universe/vehicle/autoware_external_cmd_converter /autoware/src/universe/autoware_universe/vehicle/autoware_external_cmd_converter
COPY src/universe/autoware_universe/vehicle/autoware_raw_vehicle_cmd_converter /autoware/src/universe/autoware_universe/vehicle/autoware_raw_vehicle_cmd_converter
COPY src/universe/autoware_universe/system/autoware_command_mode_types /autoware/src/universe/autoware_universe/system/autoware_command_mode_types

# diffusion_planner depends on cuda
COPY src/universe/autoware_universe/sensing/autoware_cuda_utils /autoware/src/universe/autoware_universe/sensing/autoware_cuda_utils
COPY src/universe/autoware_universe/perception/autoware_tensorrt_common /autoware/src/universe/autoware_universe/perception/autoware_tensorrt_common
COPY src/universe/autoware_universe/perception/autoware_tensorrt_plugins /autoware/src/universe/autoware_universe/perception/autoware_tensorrt_plugins

RUN /autoware/resolve_rosdep_keys.sh /autoware/src "${ROS_DISTRO}" \
  > /rosdep-universe-planning-control-depend-packages.txt \
  && cat /rosdep-universe-planning-control-depend-packages.txt
RUN /autoware/resolve_rosdep_keys.sh /autoware/src "${ROS_DISTRO}" --dependency-types=exec \
  > /rosdep-universe-planning-control-exec-depend-packages.txt \
  && cat /rosdep-universe-planning-control-exec-depend-packages.txt

FROM rosdep-depend AS rosdep-universe-vehicle-system-depend
SHELL ["/bin/bash", "-o", "pipefail", "-c"]
ARG ROS_DISTRO

COPY src/launcher/autoware_launch/tier4_universe_launch/tier4_vehicle_launch /autoware/src/launcher/autoware_launch/tier4_universe_launch/tier4_vehicle_launch
COPY src/launcher/autoware_launch/tier4_universe_launch/tier4_system_launch /autoware/src/launcher/autoware_launch/tier4_universe_launch/tier4_system_launch
COPY src/universe/autoware_universe/vehicle /autoware/src/universe/autoware_universe/vehicle
COPY src/universe/autoware_universe/system /autoware/src/universe/autoware_universe/system

# planning_validator and its dependencies
COPY src/universe/autoware_universe/planning/planning_validator /autoware/src/universe/autoware_universe/planning/planning_validator
COPY src/core/autoware_core/planning/autoware_planning_factor_interface /autoware/src/core/autoware_core/planning/autoware_planning_factor_interface
COPY src/core/autoware_core/planning/autoware_route_handler /autoware/src/core/autoware_core/planning/autoware_route_handler

COPY src/core/autoware_core/map/autoware_map_height_fitter /autoware/src/core/autoware_core/map/autoware_map_height_fitter
COPY src/universe/autoware_universe/localization/autoware_pose2twist /autoware/src/universe/autoware_universe/localization/autoware_pose2twist
COPY src/sensor_component/external/sensor_component_description /autoware/src/sensor_component/external/sensor_component_description
RUN /autoware/resolve_rosdep_keys.sh /autoware/src "${ROS_DISTRO}" \
  > /rosdep-universe-vehicle-system-depend-packages.txt \
  && cat /rosdep-universe-vehicle-system-depend-packages.txt
RUN /autoware/resolve_rosdep_keys.sh /autoware/src "${ROS_DISTRO}" --dependency-types=exec \
  > /rosdep-universe-vehicle-system-exec-depend-packages.txt \
  && cat /rosdep-universe-vehicle-system-exec-depend-packages.txt

FROM rosdep-depend AS rosdep-universe-api-depend
SHELL ["/bin/bash", "-o", "pipefail", "-c"]
ARG ROS_DISTRO

COPY src/launcher/autoware_launch/tier4_universe_launch/tier4_autoware_api_launch /autoware/src/launcher/autoware_launch/tier4_universe_launch/tier4_autoware_api_launch
COPY src/universe/autoware_universe/system/autoware_default_adapi_universe /autoware/src/universe/autoware_universe/system/autoware_default_adapi_universe
COPY src/universe/autoware_universe/evaluator/autoware_evaluation_adapter /autoware/src/universe/autoware_universe/evaluator/autoware_evaluation_adapter
COPY src/core/autoware_core/api/autoware_adapi_adaptors /autoware/src/core/autoware_core/api/autoware_adapi_adaptors
COPY src/universe/autoware_universe/system/autoware_diagnostic_graph_utils /autoware/src/universe/autoware_universe/system/autoware_diagnostic_graph_utils
COPY src/core/autoware_core/map/autoware_map_height_fitter /autoware/src/core/autoware_core/map/autoware_map_height_fitter

RUN /autoware/resolve_rosdep_keys.sh /autoware/src ${ROS_DISTRO} \
  > /rosdep-universe-api-depend-packages.txt \
  && cat /rosdep-universe-api-depend-packages.txt
RUN /autoware/resolve_rosdep_keys.sh /autoware/src ${ROS_DISTRO} --dependency-types=exec \
  > /rosdep-universe-api-exec-depend-packages.txt \
  && cat /rosdep-universe-api-exec-depend-packages.txt

FROM rosdep-depend AS rosdep-universe-depend
SHELL ["/bin/bash", "-o", "pipefail", "-c"]
ARG ROS_DISTRO

COPY src/launcher /autoware/src/launcher
COPY src/sensor_component /autoware/src/sensor_component
COPY src/universe /autoware/src/universe
RUN /autoware/resolve_rosdep_keys.sh /autoware/src "${ROS_DISTRO}" \
  > /rosdep-universe-depend-packages.txt \
  && cat /rosdep-universe-depend-packages.txt

RUN /autoware/resolve_rosdep_keys.sh /autoware/src "${ROS_DISTRO}" --dependency-types=exec \
  > /rosdep-exec-depend-packages.txt \
  && cat /rosdep-exec-depend-packages.txt

# hadolint ignore=DL3006
FROM $AUTOWARE_BASE_IMAGE AS core-common-devel
SHELL ["/bin/bash", "-o", "pipefail", "-c"]
ARG ROS_DISTRO
ENV CCACHE_DIR="/root/.ccache"

COPY docker/scripts/build_and_clean.sh /autoware/build_and_clean.sh
RUN chmod +x /autoware/build_and_clean.sh

# Set up development environment and tools
RUN --mount=type=ssh \
  --mount=type=cache,target=/var/cache/apt,sharing=locked \
  ./setup-dev-env.sh -y --module all --no-nvidia --no-cuda-drivers openadkit --ros-distro $ROS_DISTRO \
  && ./setup-dev-env.sh -y --module dev-tools openadkit --ros-distro $ROS_DISTRO \
  && pipx uninstall ansible \
  && /autoware/cleanup_apt.sh

# Install rosdep dependencies
COPY --from=rosdep-depend /rosdep-core-common-depend-packages.txt /tmp/rosdep-core-common-depend-packages.txt
# hadolint ignore=SC2002,DL3009
RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
  apt-get update \
  && cat /tmp/rosdep-core-common-depend-packages.txt | xargs apt-get install -y --no-install-recommends \
  && /autoware/cleanup_apt.sh

RUN --mount=type=cache,target="${CCACHE_DIR}" \
  --mount=type=bind,source=src/core/autoware_adapi_msgs,target=/autoware/src/core/autoware_adapi_msgs \
  --mount=type=bind,source=src/core/autoware_cmake,target=/autoware/src/core/autoware_cmake \
  --mount=type=bind,source=src/core/autoware_internal_msgs,target=/autoware/src/core/autoware_internal_msgs \
  --mount=type=bind,source=src/core/autoware_lanelet2_extension,target=/autoware/src/core/autoware_lanelet2_extension \
  --mount=type=bind,source=src/core/autoware_msgs,target=/autoware/src/core/autoware_msgs \
  --mount=type=bind,source=src/core/autoware_utils,target=/autoware/src/core/autoware_utils \
  source /opt/ros/"$ROS_DISTRO"/setup.bash \
  && /autoware/build_and_clean.sh "${CCACHE_DIR}" /opt/autoware

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["/bin/bash"]

FROM core-common-devel AS core-devel
SHELL ["/bin/bash", "-o", "pipefail", "-c"]
ARG ROS_DISTRO
ENV CCACHE_DIR="/root/.ccache"

# Install rosdep dependencies
COPY --from=rosdep-depend /rosdep-core-depend-packages.txt /tmp/rosdep-core-depend-packages.txt
# hadolint ignore=SC2002,DL3009
RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
  apt-get update \
  && cat /tmp/rosdep-core-depend-packages.txt | xargs apt-get install -y --no-install-recommends \
  && /autoware/cleanup_apt.sh

RUN --mount=type=cache,target="${CCACHE_DIR}" \
  --mount=type=bind,source=src/core/autoware_core,target=/autoware/src/core/autoware_core \
  source /opt/ros/"$ROS_DISTRO"/setup.bash \
  && /autoware/build_and_clean.sh "${CCACHE_DIR}" /opt/autoware

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["/bin/bash"]

FROM core-devel AS universe-common-devel
SHELL ["/bin/bash", "-o", "pipefail", "-c"]
ARG ROS_DISTRO
ENV CCACHE_DIR="/root/.ccache"

# Install rosdep dependencies
COPY --from=rosdep-depend /rosdep-universe-common-depend-packages.txt /tmp/rosdep-universe-common-depend-packages.txt
# hadolint ignore=SC2002,DL3009
RUN --mount=type=ssh \
  --mount=type=cache,target=/var/cache/apt,sharing=locked \
  apt-get update \
  && cat /tmp/rosdep-universe-common-depend-packages.txt | xargs apt-get install -y --no-install-recommends \
  && /autoware/cleanup_apt.sh

# hadolint ignore=SC1091
RUN --mount=type=cache,target="${CCACHE_DIR}" \
  --mount=type=bind,source=src/universe/autoware_universe/common,target=/autoware/src/universe/autoware_universe/common \
  --mount=type=bind,source=src/universe/external/eagleye,target=/autoware/src/universe/external/eagleye \
  --mount=type=bind,source=src/universe/external/glog,target=/autoware/src/universe/external/glog \
  --mount=type=bind,source=src/universe/external/llh_converter,target=/autoware/src/universe/external/llh_converter \
  --mount=type=bind,source=src/universe/external/managed_transform_buffer,target=/autoware/src/universe/external/managed_transform_buffer \
  --mount=type=bind,source=src/universe/external/morai_msgs,target=/autoware/src/universe/external/morai_msgs \
  --mount=type=bind,source=src/universe/external/muSSP,target=/autoware/src/universe/external/muSSP \
  --mount=type=bind,source=src/universe/external/pointcloud_to_laserscan,target=/autoware/src/universe/external/pointcloud_to_laserscan \
  --mount=type=bind,source=src/universe/external/rtklib_ros_bridge,target=/autoware/src/universe/external/rtklib_ros_bridge \
  --mount=type=bind,source=src/universe/external/tier4_autoware_msgs,target=/autoware/src/universe/external/tier4_autoware_msgs \
  --mount=type=bind,source=src/middleware/external,target=/autoware/src/middleware/external \
  --mount=type=bind,source=src/launcher,target=/autoware/src/launcher \
  source /opt/ros/"$ROS_DISTRO"/setup.bash \
  && source /opt/autoware/setup.bash \
  && /autoware/build_and_clean.sh "${CCACHE_DIR}" /opt/autoware

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["/bin/bash"]

FROM universe-common-devel AS universe-common-devel-cuda
SHELL ["/bin/bash", "-o", "pipefail", "-c"]

# Set up CUDA development environment
# hadolint ignore=SC2002,DL3009
RUN --mount=type=ssh \
  ./setup-dev-env.sh -y --module all --no-cuda-drivers openadkit --ros-distro "$ROS_DISTRO" \
  && pipx uninstall ansible \
  && /autoware/cleanup_apt.sh true

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["/bin/bash"]

FROM universe-common-devel AS universe-sensing-perception-devel
SHELL ["/bin/bash", "-o", "pipefail", "-c"]
ARG ROS_DISTRO
ENV CCACHE_DIR="/root/.ccache"

# Install rosdep dependencies
COPY --from=rosdep-universe-sensing-perception-depend /rosdep-universe-sensing-perception-depend-packages.txt /tmp/rosdep-universe-sensing-perception-depend-packages.txt
# hadolint ignore=SC2002,DL3009
RUN --mount=type=ssh \
  --mount=type=cache,target=/var/cache/apt,sharing=locked \
  apt-get update \
  && cat /tmp/rosdep-universe-sensing-perception-depend-packages.txt | xargs apt-get install -y --no-install-recommends \
  && /autoware/cleanup_apt.sh

# hadolint ignore=SC1091
RUN --mount=type=cache,target="${CCACHE_DIR}" \
  --mount=type=bind,source=src/launcher/autoware_launch/tier4_universe_launch/tier4_perception_launch,target=/autoware/src/launcher/autoware_launch/tier4_universe_launch/tier4_perception_launch \
  --mount=type=bind,source=src/launcher/autoware_launch/tier4_universe_launch/tier4_sensing_launch,target=/autoware/src/launcher/autoware_launch/tier4_universe_launch/tier4_sensing_launch \
  --mount=type=bind,source=src/universe/external/trt_batched_nms,target=/autoware/src/universe/external/trt_batched_nms \
  --mount=type=bind,source=src/universe/autoware_universe/perception,target=/autoware/src/universe/autoware_universe/perception \
  --mount=type=bind,source=src/universe/autoware_universe/sensing,target=/autoware/src/universe/autoware_universe/sensing \
  --mount=type=bind,source=src/universe/autoware_universe/evaluator/autoware_perception_online_evaluator,target=/autoware/src/universe/autoware_universe/evaluator/autoware_perception_online_evaluator \
  --mount=type=bind,source=src/sensor_component,target=/autoware/src/sensor_component \
  source /opt/ros/"$ROS_DISTRO"/setup.bash \
  && source /opt/autoware/setup.bash \
  && /autoware/build_and_clean.sh "${CCACHE_DIR}" /opt/autoware

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["/bin/bash"]

FROM universe-common-devel-cuda AS universe-sensing-perception-devel-cuda
SHELL ["/bin/bash", "-o", "pipefail", "-c"]
ARG ROS_DISTRO
ENV CCACHE_DIR="/root/.ccache"

# Install rosdep dependencies
COPY --from=rosdep-universe-sensing-perception-depend /rosdep-universe-sensing-perception-depend-packages.txt /tmp/rosdep-universe-sensing-perception-depend-packages.txt
# hadolint ignore=SC2002,DL3009
RUN --mount=type=ssh \
  --mount=type=cache,target=/var/cache/apt,sharing=locked \
  apt-get update \
  && cat /tmp/rosdep-universe-sensing-perception-depend-packages.txt | xargs apt-get install -y --no-install-recommends \
  && /autoware/cleanup_apt.sh

COPY --from=universe-sensing-perception-devel /opt/autoware /opt/autoware

# hadolint ignore=SC1091
RUN --mount=type=cache,target="${CCACHE_DIR}" \
  --mount=type=bind,source=src/universe/external/negotiated,target=/autoware/src/universe/external/negotiated \
  --mount=type=bind,source=src/universe/external/cuda_blackboard,target=/autoware/src/universe/external/cuda_blackboard \
  --mount=type=bind,source=src/universe/autoware_universe/perception,target=/autoware/src/universe/autoware_universe/perception \
  --mount=type=bind,source=src/universe/autoware_universe/sensing,target=/autoware/src/universe/autoware_universe/sensing \
  --mount=type=bind,source=src/universe/autoware_universe/common/autoware_cuda_dependency_meta,target=/autoware/src/universe/autoware_universe/common/autoware_cuda_dependency_meta \
  source /opt/ros/"$ROS_DISTRO"/setup.bash \
  && source /opt/autoware/setup.bash \
  && /autoware/build_and_clean.sh "${CCACHE_DIR}" /opt/autoware "--packages-above-and-dependencies autoware_cuda_dependency_meta"

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["/bin/bash"]

FROM universe-common-devel AS universe-localization-mapping-devel
SHELL ["/bin/bash", "-o", "pipefail", "-c"]
ARG ROS_DISTRO
ENV CCACHE_DIR="/root/.ccache"

# Install rosdep dependencies
COPY --from=rosdep-universe-localization-mapping-depend /rosdep-universe-localization-mapping-depend-packages.txt /tmp/rosdep-universe-localization-mapping-depend-packages.txt
# hadolint ignore=SC2002,DL3009
RUN --mount=type=ssh \
  --mount=type=cache,target=/var/cache/apt,sharing=locked \
  apt-get update \
  && cat /tmp/rosdep-universe-localization-mapping-depend-packages.txt | xargs apt-get install -y --no-install-recommends \
  && /autoware/cleanup_apt.sh

# hadolint ignore=SC1091
RUN --mount=type=cache,target="${CCACHE_DIR}" \
  --mount=type=bind,source=src/launcher/autoware_launch/tier4_universe_launch/tier4_localization_launch,target=/autoware/src/launcher/autoware_launch/tier4_universe_launch/tier4_localization_launch \
  --mount=type=bind,source=src/launcher/autoware_launch/tier4_universe_launch/tier4_map_launch,target=/autoware/src/launcher/autoware_launch/tier4_universe_launch/tier4_map_launch \
  --mount=type=bind,source=src/universe/autoware_universe/localization,target=/autoware/src/universe/autoware_universe/localization \
  --mount=type=bind,source=src/universe/autoware_universe/map,target=/autoware/src/universe/autoware_universe/map \
  # TODO(youtalk): Remove COPYs when https://github.com/autowarefoundation/autoware_universe/issues/10282 is resolved
  --mount=type=bind,source=src/universe/autoware_universe/sensing/autoware_pcl_extensions,target=/autoware/src/universe/autoware_universe/sensing/autoware_pcl_extensions \
  --mount=type=bind,source=src/universe/autoware_universe/sensing/autoware_pointcloud_preprocessor,target=/autoware/src/universe/autoware_universe/sensing/autoware_pointcloud_preprocessor \
  --mount=type=bind,source=src/universe/autoware_universe/system/autoware_default_adapi_helpers/autoware_automatic_pose_initializer,target=/autoware/src/universe/autoware_universe/system/autoware_default_adapi_helpers/autoware_automatic_pose_initializer \
  source /opt/ros/"$ROS_DISTRO"/setup.bash \
  && source /opt/autoware/setup.bash \
  && /autoware/build_and_clean.sh "${CCACHE_DIR}" /opt/autoware

FROM universe-common-devel AS universe-planning-control-devel
SHELL ["/bin/bash", "-o", "pipefail", "-c"]
ARG ROS_DISTRO
ENV CCACHE_DIR="/root/.ccache"

# Install rosdep dependencies
COPY --from=rosdep-universe-planning-control-depend /rosdep-universe-planning-control-depend-packages.txt /tmp/rosdep-universe-planning-control-depend-packages.txt
# hadolint ignore=SC2002,DL3009
RUN --mount=type=ssh \
  --mount=type=cache,target=/var/cache/apt,sharing=locked \
  apt-get update \
  && cat /tmp/rosdep-universe-planning-control-depend-packages.txt | xargs apt-get install -y --no-install-recommends \
  && /autoware/cleanup_apt.sh

# hadolint ignore=SC1091
RUN --mount=type=cache,target="${CCACHE_DIR}" \
  --mount=type=bind,source=src/launcher/autoware_launch/tier4_universe_launch/tier4_control_launch,target=/autoware/src/launcher/autoware_launch/tier4_universe_launch/tier4_control_launch \
  --mount=type=bind,source=src/launcher/autoware_launch/tier4_universe_launch/tier4_planning_launch,target=/autoware/src/launcher/autoware_launch/tier4_universe_launch/tier4_planning_launch \
  --mount=type=bind,source=src/universe/autoware_universe/control,target=/autoware/src/universe/autoware_universe/control \
  --mount=type=bind,source=src/universe/autoware_universe/planning,target=/autoware/src/universe/autoware_universe/planning \
  # TODO(youtalk): Remove --mount options when https://github.com/autowarefoundation/autoware_universe/issues/8805 is resolved
  --mount=type=bind,source=src/universe/autoware_universe/evaluator/autoware_control_evaluator,target=/autoware/src/universe/autoware_universe/evaluator/autoware_control_evaluator \
  --mount=type=bind,source=src/universe/autoware_universe/evaluator/autoware_planning_evaluator,target=/autoware/src/universe/autoware_universe/evaluator/autoware_planning_evaluator \
  --mount=type=bind,source=src/universe/autoware_universe/sensing/autoware_pcl_extensions,target=/autoware/src/universe/autoware_universe/sensing/autoware_pcl_extensions \
  --mount=type=bind,source=src/universe/autoware_universe/sensing/autoware_pointcloud_preprocessor,target=/autoware/src/universe/autoware_universe/sensing/autoware_pointcloud_preprocessor \
  --mount=type=bind,source=src/universe/autoware_universe/vehicle/autoware_external_cmd_converter,target=/autoware/src/universe/autoware_universe/vehicle/autoware_external_cmd_converter \
  --mount=type=bind,source=src/universe/autoware_universe/vehicle/autoware_raw_vehicle_cmd_converter,target=/autoware/src/universe/autoware_universe/vehicle/autoware_raw_vehicle_cmd_converter \
  --mount=type=bind,source=src/universe/autoware_universe/system/autoware_command_mode_types,target=/autoware/src/universe/autoware_universe/system/autoware_command_mode_types \
  source /opt/ros/"$ROS_DISTRO"/setup.bash \
  && source /opt/autoware/setup.bash \
  && /autoware/build_and_clean.sh "${CCACHE_DIR}" /opt/autoware

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["/bin/bash"]

FROM universe-common-devel AS universe-vehicle-system-devel
SHELL ["/bin/bash", "-o", "pipefail", "-c"]
ARG ROS_DISTRO
ENV CCACHE_DIR="/root/.ccache"

# Install rosdep dependencies
COPY --from=rosdep-universe-vehicle-system-depend /rosdep-universe-vehicle-system-depend-packages.txt /tmp/rosdep-universe-vehicle-system-depend-packages.txt
# hadolint ignore=SC2002,DL3009
RUN --mount=type=ssh \
  --mount=type=cache,target=/var/cache/apt,sharing=locked \
  apt-get update \
  && cat /tmp/rosdep-universe-vehicle-system-depend-packages.txt | xargs apt-get install -y --no-install-recommends \
  && /autoware/cleanup_apt.sh

# hadolint ignore=SC1091
RUN --mount=type=cache,target="${CCACHE_DIR}" \
  --mount=type=bind,source=src/launcher/autoware_launch/tier4_universe_launch/tier4_vehicle_launch,target=/autoware/src/launcher/autoware_launch/tier4_universe_launch/tier4_vehicle_launch \
  --mount=type=bind,source=src/launcher/autoware_launch/tier4_universe_launch/tier4_system_launch,target=/autoware/src/launcher/autoware_launch/tier4_universe_launch/tier4_system_launch \
  --mount=type=bind,source=src/universe/autoware_universe/vehicle,target=/autoware/src/universe/autoware_universe/vehicle \
  --mount=type=bind,source=src/universe/autoware_universe/system,target=/autoware/src/universe/autoware_universe/system \
  --mount=type=bind,source=src/core/autoware_core/map/autoware_map_height_fitter,target=/autoware/src/core/autoware_core/map/autoware_map_height_fitter \
  --mount=type=bind,source=src/universe/autoware_universe/localization/autoware_pose2twist,target=/autoware/src/universe/autoware_universe/localization/autoware_pose2twist \
  --mount=type=bind,source=src/universe/autoware_universe/planning/planning_validator,target=/autoware/src/universe/autoware_universe/planning/planning_validator \
  --mount=type=bind,source=src/sensor_component/external/sensor_component_description,target=/autoware/src/sensor_component/external/sensor_component_description \
  source /opt/ros/"$ROS_DISTRO"/setup.bash \
  && source /opt/autoware/setup.bash \
  && /autoware/build_and_clean.sh "${CCACHE_DIR}" /opt/autoware

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["/bin/bash"]

FROM universe-common-devel AS universe-visualization-devel
SHELL ["/bin/bash", "-o", "pipefail", "-c"]
ARG ROS_DISTRO
ENV CCACHE_DIR="/root/.ccache"

# Install rosdep dependencies
COPY --from=rosdep-universe-visualization-depend /rosdep-universe-visualization-depend-packages.txt /tmp/rosdep-universe-visualization-depend-packages.txt
# hadolint ignore=SC2002,DL3009
RUN --mount=type=ssh \
  --mount=type=cache,target=/var/cache/apt,sharing=locked \
  apt-get update \
  && cat /tmp/rosdep-universe-visualization-depend-packages.txt | xargs apt-get install -y --no-install-recommends \
  && /autoware/cleanup_apt.sh

# hadolint ignore=SC1091
RUN --mount=type=cache,target="${CCACHE_DIR}" \
  --mount=type=bind,source=src/universe/autoware_universe/visualization,target=/autoware/src/universe/autoware_universe/visualization \
  --mount=type=bind,source=src/core/autoware_rviz_plugins,target=/autoware/src/core/autoware_rviz_plugins \
  source /opt/ros/"$ROS_DISTRO"/setup.bash \
  && source /opt/autoware/setup.bash \
  && /autoware/build_and_clean.sh "${CCACHE_DIR}" /opt/autoware

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["/bin/bash"]

FROM universe-common-devel AS universe-api-devel
SHELL ["/bin/bash", "-o", "pipefail", "-c"]
ARG ROS_DISTRO
ENV CCACHE_DIR="/root/.ccache"

# Install rosdep dependencies
COPY --from=rosdep-universe-api-depend /rosdep-universe-api-depend-packages.txt /tmp/rosdep-universe-api-depend-packages.txt
# hadolint ignore=SC2002,DL3009
RUN --mount=type=ssh \
  --mount=type=cache,target=/var/cache/apt,sharing=locked \
  apt-get update \
  && cat /tmp/rosdep-universe-api-depend-packages.txt | xargs apt-get install -y --no-install-recommends \
  && /autoware/cleanup_apt.sh

# hadolint ignore=SC1091
RUN --mount=type=cache,target=${CCACHE_DIR} \
  --mount=type=bind,source=src/launcher/autoware_launch/tier4_universe_launch/tier4_autoware_api_launch,target=/autoware/src/launcher/autoware_launch/tier4_universe_launch/tier4_autoware_api_launch \
  --mount=type=bind,source=src/universe/autoware_universe/system/autoware_default_adapi_universe,target=/autoware/src/universe/autoware_universe/system/autoware_default_adapi_universe \
  --mount=type=bind,source=src/core/autoware_core/api/autoware_adapi_adaptors,target=/autoware/src/core/autoware_core/api/autoware_adapi_adaptors \
  --mount=type=bind,source=src/universe/autoware_universe/system/autoware_diagnostic_graph_utils,target=/autoware/src/universe/autoware_universe/system/autoware_diagnostic_graph_utils \
  --mount=type=bind,source=src/core/autoware_core/map/autoware_map_height_fitter,target=/autoware/src/core/autoware_core/map/autoware_map_height_fitter \
  source /opt/ros/"$ROS_DISTRO"/setup.bash \
  && source /opt/autoware/setup.bash \
  && /autoware/build_and_clean.sh ${CCACHE_DIR} /opt/autoware

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["/bin/bash"]

FROM universe-common-devel AS universe-devel
SHELL ["/bin/bash", "-o", "pipefail", "-c"]
ARG ROS_DISTRO
ENV CCACHE_DIR="/root/.ccache"

# Install rosdep dependencies
COPY --from=rosdep-universe-depend /rosdep-universe-depend-packages.txt /tmp/rosdep-universe-depend-packages.txt
# hadolint ignore=SC2002,DL3009
RUN --mount=type=ssh \
  --mount=type=cache,target=/var/cache/apt,sharing=locked \
  apt-get update \
  && cat /tmp/rosdep-universe-depend-packages.txt | xargs apt-get install -y --no-install-recommends \
  && /autoware/cleanup_apt.sh

COPY --from=universe-sensing-perception-devel /opt/autoware /opt/autoware
COPY --from=universe-localization-mapping-devel /opt/autoware /opt/autoware
COPY --from=universe-planning-control-devel /opt/autoware /opt/autoware
COPY --from=universe-vehicle-system-devel /opt/autoware /opt/autoware
COPY --from=universe-visualization-devel /opt/autoware /opt/autoware
COPY --from=universe-api-devel /opt/autoware /opt/autoware

# strip unneeded symbols from binaries in /opt/autoware
RUN find /opt/autoware -type f \( -executable -o -name "*.so" \) -exec strip --strip-unneeded --remove-section=.comment --remove-section=.note {} \;

# hadolint ignore=SC1091
RUN --mount=type=cache,target="${CCACHE_DIR}" \
  --mount=type=bind,source=src/sensor_component,target=/autoware/src/sensor_component \
  --mount=type=bind,source=src/universe/autoware_universe/evaluator,target=/autoware/src/universe/autoware_universe/evaluator \
  --mount=type=bind,source=src/launcher/autoware_launch/tier4_universe_launch,target=/autoware/src/launcher/autoware_launch/tier4_universe_launch \
  --mount=type=bind,source=src/universe/autoware_universe/simulator,target=/autoware/src/universe/autoware_universe/simulator \
  source /opt/ros/"$ROS_DISTRO"/setup.bash \
  && source /opt/autoware/setup.bash \
  && /autoware/build_and_clean.sh "${CCACHE_DIR}" /opt/autoware

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["/bin/bash"]

FROM universe-common-devel-cuda AS universe-devel-cuda
SHELL ["/bin/bash", "-o", "pipefail", "-c"]

# Install rosdep dependencies
COPY --from=rosdep-universe-depend /rosdep-universe-depend-packages.txt /tmp/rosdep-universe-depend-packages.txt
# hadolint ignore=SC2002,DL3009
RUN --mount=type=ssh \
  --mount=type=cache,target=/var/cache/apt,sharing=locked \
  apt-get update \
  && cat /tmp/rosdep-universe-depend-packages.txt | xargs apt-get install -y --no-install-recommends \
  && /autoware/cleanup_apt.sh

COPY --from=universe-devel /opt/autoware /opt/autoware
COPY --from=universe-sensing-perception-devel-cuda /opt/autoware /opt/autoware

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["/bin/bash"]

# hadolint ignore=DL3006
FROM $AUTOWARE_BASE_IMAGE AS core
SHELL ["/bin/bash", "-o", "pipefail", "-c"]
ARG ROS_DISTRO
ARG LIB_DIR

# Set up runtime environment
COPY --from=rosdep-depend /rosdep-core-exec-depend-packages.txt /tmp/rosdep-core-exec-depend-packages.txt
# hadolint ignore=SC2002
RUN --mount=type=ssh \
  --mount=type=cache,target=/var/cache/apt,sharing=locked \
  ./setup-dev-env.sh -y --module all --no-nvidia --no-cuda-drivers --runtime openadkit --ros-distro $ROS_DISTRO \
  && pipx uninstall ansible \
  && apt-get update \
  && cat /tmp/rosdep-core-exec-depend-packages.txt | xargs apt-get install -y --no-install-recommends \
  && /autoware/cleanup_system.sh $LIB_DIR $ROS_DISTRO

COPY --from=core-devel /opt/autoware /opt/autoware

# Copy bash aliases
COPY docker/etc/.bash_aliases /root/.bash_aliases
RUN echo "source /opt/autoware/setup.bash" > /etc/bash.bashrc

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["/bin/bash"]

# hadolint ignore=DL3006
FROM $AUTOWARE_BASE_IMAGE AS universe-sensing-perception
SHELL ["/bin/bash", "-o", "pipefail", "-c"]
ARG ROS_DISTRO
ARG LIB_DIR

# Set up runtime environment
COPY --from=rosdep-universe-sensing-perception-depend /rosdep-universe-sensing-perception-exec-depend-packages.txt /tmp/rosdep-universe-sensing-perception-exec-depend-packages.txt
# hadolint ignore=SC2002
RUN --mount=type=ssh \
  --mount=type=cache,target=/var/cache/apt,sharing=locked \
  ./setup-dev-env.sh -y --module all --no-nvidia --no-cuda-drivers --runtime openadkit --ros-distro $ROS_DISTRO \
  && pipx uninstall ansible \
  && apt-get update \
  && cat /tmp/rosdep-universe-sensing-perception-exec-depend-packages.txt | xargs apt-get install -y --no-install-recommends \
  && /autoware/cleanup_system.sh $LIB_DIR $ROS_DISTRO

COPY --from=universe-sensing-perception-devel /opt/autoware /opt/autoware
# TODO(youtalk): Fix https://github.com/autowarefoundation/autoware/pull/6235 workaround
COPY --from=universe-common-devel /opt/autoware/share/autoware_launch /opt/autoware/share/autoware_launch

# Copy bash aliases
COPY docker/etc/.bash_aliases /root/.bash_aliases
RUN echo "source /opt/autoware/setup.bash" > /etc/bash.bashrc

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["/bin/bash"]

# hadolint ignore=DL3006
FROM $AUTOWARE_BASE_CUDA_IMAGE AS universe-sensing-perception-cuda
SHELL ["/bin/bash", "-o", "pipefail", "-c"]
ARG ROS_DISTRO
ARG LIB_DIR

# Set up runtime environment
COPY --from=rosdep-universe-sensing-perception-depend /rosdep-universe-sensing-perception-exec-depend-packages.txt /tmp/rosdep-universe-sensing-perception-exec-depend-packages.txt
# hadolint ignore=SC2002
RUN --mount=type=ssh \
  --mount=type=cache,target=/var/cache/apt,sharing=locked \
  ./setup-dev-env.sh -y --module all --no-nvidia --no-cuda-drivers --runtime openadkit --ros-distro $ROS_DISTRO \
  && pipx uninstall ansible \
  && apt-get update \
  && cat /tmp/rosdep-universe-sensing-perception-exec-depend-packages.txt | xargs apt-get install -y --no-install-recommends \
  && /autoware/cleanup_system.sh $LIB_DIR $ROS_DISTRO

COPY --from=universe-sensing-perception-devel-cuda /opt/autoware /opt/autoware

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["/bin/bash"]

# hadolint ignore=DL3006
FROM $AUTOWARE_BASE_IMAGE AS universe-localization-mapping
SHELL ["/bin/bash", "-o", "pipefail", "-c"]
ARG ROS_DISTRO
ARG LIB_DIR

# Set up runtime environment
COPY --from=rosdep-universe-localization-mapping-depend /rosdep-universe-localization-mapping-exec-depend-packages.txt /tmp/rosdep-universe-localization-mapping-exec-depend-packages.txt
# hadolint ignore=SC2002
RUN --mount=type=ssh \
  --mount=type=cache,target=/var/cache/apt,sharing=locked \
  ./setup-dev-env.sh -y --module all --no-nvidia --no-cuda-drivers --runtime openadkit --ros-distro $ROS_DISTRO \
  && pipx uninstall ansible \
  && apt-get update \
  && cat /tmp/rosdep-universe-localization-mapping-exec-depend-packages.txt | xargs apt-get install -y --no-install-recommends \
  && /autoware/cleanup_system.sh $LIB_DIR $ROS_DISTRO

COPY --from=universe-localization-mapping-devel /opt/autoware /opt/autoware

# Copy bash aliases
COPY docker/etc/.bash_aliases /root/.bash_aliases
RUN echo "source /opt/autoware/setup.bash" > /etc/bash.bashrc

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["/bin/bash"]

# hadolint ignore=DL3006
FROM $AUTOWARE_BASE_IMAGE AS universe-planning-control
SHELL ["/bin/bash", "-o", "pipefail", "-c"]
ARG ROS_DISTRO
ARG LIB_DIR

# Set up runtime environment
COPY --from=rosdep-universe-planning-control-depend /rosdep-universe-planning-control-exec-depend-packages.txt /tmp/rosdep-universe-planning-control-exec-depend-packages.txt
# hadolint ignore=SC2002
RUN --mount=type=ssh \
  --mount=type=cache,target=/var/cache/apt,sharing=locked \
  ./setup-dev-env.sh -y --module all --no-nvidia --no-cuda-drivers --runtime openadkit --ros-distro $ROS_DISTRO \
  && pipx uninstall ansible \
  && apt-get update \
  && cat /tmp/rosdep-universe-planning-control-exec-depend-packages.txt | xargs apt-get install -y --no-install-recommends \
  && /autoware/cleanup_system.sh $LIB_DIR $ROS_DISTRO

COPY --from=universe-planning-control-devel /opt/autoware /opt/autoware

# Copy bash aliases
COPY docker/etc/.bash_aliases /root/.bash_aliases
RUN echo "source /opt/autoware/setup.bash" > /etc/bash.bashrc

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["/bin/bash"]

# hadolint ignore=DL3006
FROM $AUTOWARE_BASE_IMAGE AS universe-vehicle-system
SHELL ["/bin/bash", "-o", "pipefail", "-c"]
ARG ROS_DISTRO
ARG LIB_DIR

# Set up runtime environment
COPY --from=rosdep-universe-vehicle-system-depend /rosdep-universe-vehicle-system-exec-depend-packages.txt /tmp/rosdep-universe-vehicle-system-exec-depend-packages.txt
# hadolint ignore=SC2002
RUN --mount=type=ssh \
  --mount=type=cache,target=/var/cache/apt,sharing=locked \
  ./setup-dev-env.sh -y --module all --no-nvidia --no-cuda-drivers --runtime openadkit --ros-distro $ROS_DISTRO \
  && pipx uninstall ansible \
  && apt-get update \
  && cat /tmp/rosdep-universe-vehicle-system-exec-depend-packages.txt | xargs apt-get install -y --no-install-recommends \
  && /autoware/cleanup_system.sh $LIB_DIR $ROS_DISTRO

COPY --from=universe-vehicle-system-devel /opt/autoware /opt/autoware

# Copy bash aliases
COPY docker/etc/.bash_aliases /root/.bash_aliases
RUN echo "source /opt/autoware/setup.bash" > /etc/bash.bashrc

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["/bin/bash"]

# hadolint ignore=DL3006
FROM $AUTOWARE_BASE_IMAGE AS universe-visualization
SHELL ["/bin/bash", "-o", "pipefail", "-c"]
ARG ROS_DISTRO
ARG LIB_DIR

# Set up runtime environment
COPY --from=rosdep-universe-visualization-depend /rosdep-universe-visualization-exec-depend-packages.txt /tmp/rosdep-universe-visualization-exec-depend-packages.txt
# hadolint ignore=SC2002
RUN --mount=type=ssh \
  --mount=type=cache,target=/var/cache/apt,sharing=locked \
  ./setup-dev-env.sh -y --module all --no-nvidia --no-cuda-drivers --runtime openadkit --ros-distro $ROS_DISTRO \
  && pipx uninstall ansible \
  && apt-get update \
  && cat /tmp/rosdep-universe-visualization-exec-depend-packages.txt | xargs apt-get install -y --no-install-recommends \
  && /autoware/cleanup_system.sh $LIB_DIR $ROS_DISTRO

COPY --from=universe-visualization-devel /opt/autoware /opt/autoware

# Copy bash aliases
COPY docker/etc/.bash_aliases /root/.bash_aliases
RUN echo "source /opt/autoware/setup.bash" > /etc/bash.bashrc

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["/bin/bash"]

# hadolint ignore=DL3006
FROM $AUTOWARE_BASE_IMAGE AS universe-api
SHELL ["/bin/bash", "-o", "pipefail", "-c"]
ARG ROS_DISTRO
ARG LIB_DIR

# Set up runtime environment
COPY --from=rosdep-universe-api-depend /rosdep-universe-api-exec-depend-packages.txt /tmp/rosdep-universe-api-exec-depend-packages.txt
# hadolint ignore=SC2002
RUN --mount=type=ssh \
  --mount=type=cache,target=/var/cache/apt,sharing=locked \
  ./setup-dev-env.sh -y --module all --no-nvidia --no-cuda-drivers --runtime openadkit --ros-distro $ROS_DISTRO \
  && pipx uninstall ansible \
  && apt-get update \
  && cat /tmp/rosdep-universe-api-exec-depend-packages.txt | xargs apt-get install -y --no-install-recommends \
  && /autoware/cleanup_system.sh $LIB_DIR $ROS_DISTRO

COPY --from=universe-api-devel /opt/autoware /opt/autoware

# Copy bash aliases
COPY docker/etc/.bash_aliases /root/.bash_aliases
RUN echo "source /opt/autoware/setup.bash" > /etc/bash.bashrc

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["/bin/bash"]

# hadolint ignore=DL3006
FROM $AUTOWARE_BASE_IMAGE AS universe
SHELL ["/bin/bash", "-o", "pipefail", "-c"]
ARG ROS_DISTRO
ARG LIB_DIR

# Set up runtime environment
COPY --from=rosdep-universe-depend /rosdep-exec-depend-packages.txt /tmp/rosdep-exec-depend-packages.txt
# hadolint ignore=SC2002,DL3009
RUN --mount=type=ssh \
  --mount=type=cache,target=/var/cache/apt,sharing=locked \
  ./setup-dev-env.sh -y --module all --no-nvidia --no-cuda-drivers --runtime openadkit --ros-distro $ROS_DISTRO \
  && pipx uninstall ansible \
  && apt-get update \
  && cat /tmp/rosdep-exec-depend-packages.txt | xargs apt-get install -y --no-install-recommends \
  && /autoware/cleanup_system.sh $LIB_DIR $ROS_DISTRO

COPY --from=universe-devel /opt/autoware /opt/autoware

# Copy bash aliases
COPY docker/etc/.bash_aliases /root/.bash_aliases
RUN echo "source /opt/autoware/setup.bash" > /etc/bash.bashrc

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["/bin/bash"]

# hadolint ignore=DL3006
FROM $AUTOWARE_BASE_CUDA_IMAGE AS universe-cuda
SHELL ["/bin/bash", "-o", "pipefail", "-c"]
ARG ROS_DISTRO
ARG LIB_DIR

# Set up runtime environment
COPY --from=rosdep-universe-depend /rosdep-exec-depend-packages.txt /tmp/rosdep-exec-depend-packages.txt
# hadolint ignore=SC2002,DL3009
RUN --mount=type=ssh \
  --mount=type=cache,target=/var/cache/apt,sharing=locked \
  ./setup-dev-env.sh -y --module all --no-nvidia --no-cuda-drivers --runtime openadkit --ros-distro $ROS_DISTRO \
  && pipx uninstall ansible \
  && apt-get update \
  && cat /tmp/rosdep-exec-depend-packages.txt | xargs apt-get install -y --no-install-recommends \
  && /autoware/cleanup_system.sh $LIB_DIR $ROS_DISTRO

COPY --from=universe-devel-cuda /opt/autoware /opt/autoware

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["/bin/bash"]
