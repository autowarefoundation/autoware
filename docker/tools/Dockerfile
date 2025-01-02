ARG ROS_DISTRO

### Builder
FROM ghcr.io/autowarefoundation/autoware:universe-devel AS builder
SHELL ["/bin/bash", "-o", "pipefail", "-c"]
ENV CCACHE_DIR="/root/.ccache"
WORKDIR /autoware
COPY src /autoware/src
COPY simulator.repos /autoware/simulator.repos
COPY docker/scripts/resolve_rosdep_keys.sh /autoware/resolve_rosdep_keys.sh
RUN chmod +x /autoware/resolve_rosdep_keys.sh

# Install dependencies and build the simulator
RUN --mount=type=ssh \
    --mount=type=cache,target=/var/cache/apt,sharing=locked \
    vcs import src < simulator.repos \
    && apt-get update \
    && rosdep update && rosdep install -y --from-paths src --ignore-src --rosdistro $ROS_DISTRO \
    && source /opt/ros/"$ROS_DISTRO"/setup.bash && source /opt/autoware/setup.bash \
    && colcon build --cmake-args \
      "-Wno-dev" \
      "--no-warn-unused-cli" \
      --install-base /opt/autoware \
      --merge-install \
      --mixin release compile-commands ccache \
      --base-paths /autoware/src/simulator \
    && find /opt/autoware/lib -type f -name "*.py" -exec chmod +x {} \; \
    && find /opt/autoware/share -type f -name "*.py" -exec chmod +x {} \; \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/* "$HOME"/.cache

# Extract rosdep dependencies for simulator
RUN /autoware/resolve_rosdep_keys.sh /autoware/src/simulator ${ROS_DISTRO} \
    > /rosdep-simulator-depend-packages.txt \
    && cat /rosdep-simulator-depend-packages.txt

### Simulator
FROM ghcr.io/autowarefoundation/autoware:universe AS simulator
WORKDIR /autoware
COPY --from=builder /opt/autoware /opt/autoware
COPY --from=builder /rosdep-simulator-depend-packages.txt /tmp/rosdep-simulator-depend-packages.txt

RUN --mount=type=ssh \
    --mount=type=cache,target=/var/cache/apt,sharing=locked \
    apt-get update && apt-get install -y curl unzip \
    && source /opt/ros/"$ROS_DISTRO"/setup.bash && source /opt/autoware/setup.bash \
    && rosdep update \
    # Remove xmlschema and yamale from rosdep packages since we install via pip
    && sed -i '/\(xmlschema\|yamale\)/d' /tmp/rosdep-simulator-depend-packages.txt \
    && pip install yamale xmlschema \
    && cat /tmp/rosdep-simulator-depend-packages.txt | xargs apt-get install -y --no-install-recommends \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/* "$HOME"/.cache && \
    echo "source /opt/autoware/setup.bash" > /etc/bash.bashrc

COPY docker/tools/etc/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
CMD ["/bin/bash"]

### Visualizer
FROM simulator AS visualizer
WORKDIR /autoware

# Install openbox and VNC requirements
RUN apt-get update && DEBIAN_FRONTEND=noninteractive apt-get install -y \
    curl unzip openbox tigervnc-standalone-server tigervnc-common \
    novnc websockify python3-numpy python3-xdg \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*

# Set up VNC password
RUN mkdir -p ~/.vnc && \
    echo "openadkit" | vncpasswd -f > ~/.vnc/passwd && \
    chmod 600 ~/.vnc/passwd

# Create SSL certificate for NoVNC
RUN openssl req -x509 -nodes -newkey rsa:2048 \
    -keyout /etc/ssl/private/novnc.key \
    -out /etc/ssl/certs/novnc.crt \
    -days 365 \
    -subj "/O=Autoware-OpenADKit/CN=localhost"

# Install ngrok for optional public access if no public ip available
RUN curl -sSL https://ngrok-agent.s3.amazonaws.com/ngrok.asc \
    | tee /etc/apt/trusted.gpg.d/ngrok.asc >/dev/null && \
    echo "deb https://ngrok-agent.s3.amazonaws.com buster main" \
    | tee /etc/apt/sources.list.d/ngrok.list && \
    apt update && \
    apt install ngrok

# Need to expose VNC and NoVNC ports when running the container
EXPOSE 5900 6080

# Add source commands to bash startup
RUN echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> /root/.bashrc && \
    echo "source /opt/autoware/setup.bash" >> /root/.bashrc

# Copy startup scripts
COPY docker/tools/etc/xstartup /root/.vnc/xstartup
COPY docker/tools/etc/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh && chmod +x /root/.vnc/xstartup
ENTRYPOINT ["/entrypoint.sh"]
CMD ["/bin/bash"]