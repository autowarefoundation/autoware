# cspell:ignore VNC, openbox, dbus-x11, tigervnc, novnc, websockify, newkey, keyout
FROM ubuntu:22.04 AS visualizer
SHELL ["/bin/bash", "-o", "pipefail", "-c"]

# Install VNC dependencies
# hadolint ignore=SC2002
RUN apt-get update && DEBIAN_FRONTEND=noninteractive apt-get install -y \
  curl \
  unzip \
  tigervnc-standalone-server \
  tigervnc-common \
  novnc \
  websockify \
  python3-numpy \
  python3-xdg \
  openbox \
  x11-xserver-utils \
  xauth \
  dbus-x11 \
  && rm -rf /var/lib/apt/lists/*

# Create SSL certificate for NoVNC
RUN openssl req -x509 -nodes -newkey rsa:2048 \
  -keyout /etc/ssl/private/novnc.key \
  -out /etc/ssl/certs/novnc.crt \
  -days 365 \
  -subj "/O=Autoware-OpenADKit/CN=localhost"

# Need to expose NoVNC port when running the container
EXPOSE 6080

# Copy startup script
COPY docker/tools/visualizer/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
