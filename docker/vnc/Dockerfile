FROM ghcr.io/autowarefoundation/autoware:universe AS vnc

# Install VNC server with NoVNC
RUN apt-get update && apt-get install -y \
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
