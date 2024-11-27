FROM ghcr.io/autowarefoundation/autoware:universe-devel AS builder
WORKDIR /autoware

# Copy scenario_simulator and pull external dependencies
COPY src/simulator /autoware/src/simulator
RUN mkdir -p src/simulator/scenario_simulator/external \
  && vcs import src/simulator/scenario_simulator/external < src/simulator/scenario_simulator/dependency_${ROS_DISTRO}.repos

# Install rosdep dependencies
RUN --mount=type=ssh \
  apt-get update \
  && rosdep update \
  && rosdep install -y --from-paths src --ignore-src --rosdistro $ROS_DISTRO \
  && apt-get autoremove -y && rm -rf "$HOME"/.cache

# Build simulator
RUN source /opt/ros/"$ROS_DISTRO"/setup.bash \
  && source /opt/autoware/setup.bash \
  && colcon build --cmake-args \
  " -Wno-dev" \
  " --no-warn-unused-cli" \
  --install-base /opt/autoware \
  --merge-install \
  --mixin release compile-commands ccache \
  --packages-skip-regex ".*autoware.*" \
  && rm -rf build/ install/ log/

RUN find /opt/autoware/lib -type f -name "*.py" -exec chmod +x {} \;
RUN find /opt/autoware/share -type f -name "*.py" -exec chmod +x {} \;

FROM ghcr.io/autowarefoundation/autoware:universe as simulator-visualizer

COPY --from=builder /opt/autoware /opt/autoware
COPY --from=builder /autoware /autoware

# Install rosdep dependencies
RUN --mount=type=ssh \
  apt-get update \
  && rosdep update \
  && rosdep install -y --from-paths src --ignore-src --dependency-types=exec --rosdistro $ROS_DISTRO \
  && apt-get autoremove -y && rm -rf "$HOME"/.cache

# Install VNC server with NoVNC
RUN apt-get update && apt-get install -y \
    curl \
    unzip \
    lxde \
    tigervnc-standalone-server \
    tigervnc-common \
    novnc \
    websockify \
    python3-numpy \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*

# Set up VNC password
RUN mkdir -p ~/.vnc && \
    echo "openadkit" | vncpasswd -f > ~/.vnc/passwd && \
    chmod 600 ~/.vnc/passwd

# Copy xstartup script
RUN mkdir -p /root/.vnc
COPY docker/vnc/xstartup /root/.vnc/xstartup

# Set permissions for xstartup script
RUN chmod +x /root/.vnc/xstartup

# Create SSL certificate for NoVNC
RUN openssl req -x509 -nodes -newkey rsa:2048 \
    -keyout /etc/ssl/private/novnc.key \
    -out /etc/ssl/certs/novnc.crt \
    -days 365 \
    -subj "/O=Autoware-OpenADKit/CN=localhost"

# Install ngrok
RUN curl -s https://ngrok-agent.s3.amazonaws.com/ngrok.asc | \
  gpg --dearmor -o /etc/apt/keyrings/ngrok.gpg && \
  echo "deb [signed-by=/etc/apt/keyrings/ngrok.gpg] https://ngrok-agent.s3.amazonaws.com buster main" | \
  tee /etc/apt/sources.list.d/ngrok.list && \
  apt update && apt install ngrok

# Need to expose VNC and NoVNC ports when running the container
EXPOSE 5900 6080

# Copy entrypoint script
COPY docker/vnc/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
