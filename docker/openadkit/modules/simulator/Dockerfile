FROM ghcr.io/autowarefoundation/autoware:latest-base as simulator
SHELL ["/bin/bash", "-o", "pipefail", "-c"]
ARG PLATFORM
ARG LIB_DIR
ARG ROS_DISTRO

COPY simulator.repos /autoware/
COPY autoware.repos /autoware/

RUN --mount=type=ssh \
  mkdir src \
  && vcs import src < simulator.repos \
  && vcs import src/simulator/scenario_simulator/external < src/simulator/scenario_simulator/dependency_humble.repos \
  && apt-get update \
  && rosdep update \
  && source /opt/ros/"$ROS_DISTRO"/setup.bash \
  && DEBIAN_FRONTEND=noninteractive rosdep install -y -t exec --rosdistro "$ROS_DISTRO" --from-paths src --ignore-src \
  && colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release \
  && find /autoware/install -type d -exec chmod -R 777 {} \; \
  && rm -rf src/* \
  && vcs import src < autoware.repos \
  && apt-get update \
  && rosdep update \
  && DEBIAN_FRONTEND=noninteractive rosdep install -y --rosdistro "$ROS_DISTRO" --from-paths src --ignore-src \
  && apt-get autoremove -y && apt-get clean -y && rm -rf /var/lib/apt/lists/* "$HOME"/.cache \
  && find /usr/lib/$LIB_DIR-linux-gnu -name "*.a" -type f -delete \
  && find / -name "*.o" -type f -delete \
  && find / -name "*.h" -type f -delete \
  && find / -name "*.hpp" -type f -delete \
  && rm -rf /autoware/src /autoware/ansible /autoware/simulator.repos \
    /root/.local/pipx /opt/ros/"$ROS_DISTRO"/include /etc/apt/sources.list.d/cuda*.list \
    /etc/apt/sources.list.d/docker.list /etc/apt/sources.list.d/nvidia-docker.list \
    /usr/include /usr/share/doc /usr/lib/gcc /usr/lib/jvm /usr/lib/llvm*

COPY --from=ghcr.io/autowarefoundation/autoware:latest-prebuilt /autoware/install /autoware/install

# Create entrypoint
COPY docker/openadkit/etc/ros_entrypoint.sh /ros_entrypoint.sh
RUN chmod +x /ros_entrypoint.sh
ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
