# cspell:ignore openbox, VNC, tigervnc, novnc, websockify, newkey, xstartup, keyout

### Builder
FROM ghcr.io/autowarefoundation/autoware:universe-visualization-devel AS builder
SHELL ["/bin/bash", "-o", "pipefail", "-c"]
ENV CCACHE_DIR="/root/.ccache"
ARG ROS_DISTRO
WORKDIR /autoware
COPY src/simulator /autoware/src/simulator
COPY docker/scripts/resolve_rosdep_keys.sh /autoware/resolve_rosdep_keys.sh
RUN chmod +x /autoware/resolve_rosdep_keys.sh

# Build simulator messages and rviz plugins ONLY
# hadolint ignore=SC1091
RUN --mount=type=ssh \
  --mount=type=cache,target=/var/cache/apt,sharing=locked \
  apt-get update \
  && source /opt/ros/"$ROS_DISTRO"/setup.bash && source /opt/autoware/setup.bash \
  && rosdep update && rosdep install -y --from-paths src --ignore-src --rosdistro $ROS_DISTRO \
  && colcon build --cmake-args \
    "-Wno-dev" \
    "--no-warn-unused-cli" \
    --install-base /opt/autoware \
    --merge-install \
    --mixin release compile-commands ccache \
    --base-paths /autoware/src \
    --packages-up-to-regex ".*_msgs$" ".*rviz_plugin$" \
  && apt-get clean \
  && rm -rf /var/lib/apt/lists/* "$HOME"/.cache

# Extract rosdep dependencies for visualizer
# hadolint ignore=SC1091
RUN source /opt/ros/"$ROS_DISTRO"/setup.bash && source /opt/autoware/setup.bash \
    && /autoware/resolve_rosdep_keys.sh /autoware/src ${ROS_DISTRO} --dependency-types=exec \
    > /rosdep-visualizer-depend-packages.txt \
    && cat /rosdep-visualizer-depend-packages.txt

FROM ghcr.io/autowarefoundation/autoware:universe AS universe

# Extract RViz configs for visualizer
# hadolint ignore=SC1091
RUN source /opt/ros/"$ROS_DISTRO"/setup.bash && source /opt/autoware/setup.bash \
    && RVIZ_PATH="$(ros2 pkg prefix --share autoware_launch)/rviz" \
    && cp -r "$RVIZ_PATH" /autoware/rviz

### visualizer
FROM ghcr.io/autowarefoundation/autoware:universe-visualization AS visualizer
SHELL ["/bin/bash", "-o", "pipefail", "-c"]
ARG ROS_DISTRO
ARG LIB_DIR
WORKDIR /autoware

# Get simulator messages, rviz plugins and dependencies
COPY --from=builder /opt/autoware /opt/autoware
COPY --from=builder /rosdep-visualizer-depend-packages.txt /tmp/rosdep-visualizer-depend-packages.txt
COPY --from=universe /autoware/rviz /autoware/rviz

# Install openbox, VNC, and simulator dependencies
# hadolint ignore=SC2002
RUN apt-get update && DEBIAN_FRONTEND=noninteractive apt-get install -y \
  curl unzip openbox tigervnc-standalone-server tigervnc-common \
  novnc websockify python3-numpy python3-xdg \
  # Remove xmlschema and yamale from rosdep packages since we install via pip
  && sed -i '/\(xmlschema\|yamale\)/d' /tmp/rosdep-visualizer-depend-packages.txt \
  && pip install --no-cache-dir yamale xmlschema \
  && cat /tmp/rosdep-visualizer-depend-packages.txt | xargs apt-get install -y --no-install-recommends \
  && /autoware/cleanup_system.sh $LIB_DIR $ROS_DISTRO

# Create SSL certificate for NoVNC
RUN openssl req -x509 -nodes -newkey rsa:2048 \
    -keyout /etc/ssl/private/novnc.key \
    -out /etc/ssl/certs/novnc.crt \
    -days 365 \
    -subj "/O=Autoware-OpenADKit/CN=localhost"

# Need to expose VNC and NoVNC ports when running the container
EXPOSE 5900 6080

# Add source commands to bash startup
RUN echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> /root/.bashrc && \
    echo "source /opt/autoware/setup.bash" >> /root/.bashrc

# Copy startup scripts
COPY docker/tools/visualizer/xstartup /root/.vnc/xstartup
COPY docker/tools/visualizer/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh && chmod +x /root/.vnc/xstartup
ENTRYPOINT ["/entrypoint.sh"]
