# cspell:ignore openbox, VNC, tigervnc, novnc, websockify, newkey, xstartup, keyout
FROM ghcr.io/autowarefoundation/autoware:universe AS universe
SHELL ["/bin/bash", "-o", "pipefail", "-c"]

# Extract RViz configs for visualizer
# hadolint ignore=SC1091
RUN source /opt/autoware/setup.bash \
  && RVIZ_PATH="$(ros2 pkg prefix --share autoware_launch)/rviz" \
  && cp -r "$RVIZ_PATH" /autoware/rviz

### visualizer
FROM ghcr.io/autowarefoundation/autoware:universe-visualization AS visualizer
SHELL ["/bin/bash", "-o", "pipefail", "-c"]
ARG ROS_DISTRO
ARG LIB_DIR

COPY --from=universe /autoware/rviz /autoware/rviz
WORKDIR /autoware

# Install openbox, VNC, and simulator dependencies
# hadolint ignore=SC2002
RUN apt-get update && DEBIAN_FRONTEND=noninteractive apt-get install -y \
  curl \
  unzip \
  openbox \
  tigervnc-standalone-server \
  tigervnc-common \
  tigervnc-tools \
  novnc \
  websockify \
  python3-numpy \
  python3-xdg \
  && pip install --no-cache-dir yamale xmlschema \
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
