ARG PLATFORM
FROM ghcr.io/autowarefoundation/autoware-openadk:latest-base-${PLATFORM} as base
SHELL ["/bin/bash", "-o", "pipefail", "-c"]
ARG PLATFORM
ARG ROS_DISTRO

# Copy install folder from prebuilt
COPY --from=ghcr.io/autowarefoundation/autoware-openadk:latest-prebuilt-${PLATFORM} /autoware/install/ /autoware/install/
COPY autoware.repos /autoware/

# Dependencies
RUN --mount=type=ssh \
  ./setup-dev-env.sh -y --module planning-control --runtime openadk \
  && pip uninstall -y ansible ansible-core \
  && mkdir src \
  && vcs import src < autoware.repos \
  && apt-get update \
  && rosdep update \
  && DEBIAN_FRONTEND=noninteractive rosdep install -y --dependency-types=exec --ignore-src --from-paths src --rosdistro "$ROS_DISTRO" \
  && apt-get autoremove -y && apt-get clean -y && rm -rf /var/lib/apt/lists/* "$HOME"/.cache \
  && find /usr/lib/$LIB_DIR-linux-gnu -name "*.a" -type f -delete \
  && find / -name "*.o" -type f -delete \
  && find / -name "*.h" -type f -delete \
  && find / -name "*.hpp" -type f -delete \
  && rm -rf /autoware/src /autoware/ansible /autoware/autoware.repos \
    /root/.local/pipx /opt/ros/"$ROS_DISTRO"/include /etc/apt/sources.list.d/cuda*.list \
    /etc/apt/sources.list.d/docker.list /etc/apt/sources.list.d/nvidia-docker.list \
    /usr/include /usr/share/doc /usr/lib/gcc /usr/lib/jvm /usr/lib/llvm*

# Copy launch file
COPY docker/autoware-openadk/services/planning-control/launch/ /autoware/install/planning_control/share/planning_control/launch/

# Create entrypoint
COPY docker/autoware-openadk/etc/ros_entrypoint.sh /ros_entrypoint.sh
RUN chmod +x /ros_entrypoint.sh
ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]