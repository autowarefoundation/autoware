FROM ghcr.io/autowarefoundation/autoware-openadk:latest-base as simulator
SHELL ["/bin/bash", "-o", "pipefail", "-c"]
ARG PLATFORM
ARG ROS_DISTRO

COPY simulator.repos /autoware/

RUN --mount=type=ssh \
  mkdir src \
  && vcs import src < simulator.repos \
  && vcs import src/simulator/scenario_simulator/external < src/simulator/scenario_simulator/dependency_humble.repos \
  && apt-get update \
  && rosdep update \
  && DEBIAN_FRONTEND=noninteractive rosdep install -y --rosdistro "$ROS_DISTRO" --ignore-src --from-paths src \
  && source /opt/ros/"$ROS_DISTRO"/setup.bash \
  && colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release --parallel-workers 8 \
  && find /autoware/install -type d -exec chmod 777 {} \; \
  && apt-get autoremove -y && apt-get clean -y && rm -rf /var/lib/apt/lists/* "$HOME"/.cache \
  && find /usr/lib/$LIB_DIR-linux-gnu -name "*.a" -type f -delete \
  && find / -name "*.o" -type f -delete \
  && find / -name "*.h" -type f -delete \
  && find / -name "*.hpp" -type f -delete \
  && rm -rf /autoware/src /autoware/ansible /autoware/autoware.repos \
    /root/.local/pipx /opt/ros/"$ROS_DISTRO"/include /etc/apt/sources.list.d/cuda*.list \
    /etc/apt/sources.list.d/docker.list /etc/apt/sources.list.d/nvidia-docker.list \
    /usr/include /usr/share/doc /usr/lib/gcc /usr/lib/jvm /usr/lib/llvm*

# Copy bash aliases for simulation
# TO-DO

# Create entrypoint
COPY docker/autoware-openadk/etc/ros_entrypoint.sh /ros_entrypoint.sh
RUN chmod +x /ros_entrypoint.sh
ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
