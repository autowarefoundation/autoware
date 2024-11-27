#!/usr/bin/env bash

# Source ROS2 and Autoware
source "/opt/ros/$ROS_DISTRO/setup.bash"
source /opt/autoware/setup.bash

# Start VNC server
vncserver :1 -auth $HOME/.Xauthority -geometry 1024x768 -depth 16 -pixelformat rgb565 >/dev/null 2>&1
VNC_RESULT=$?

if [ $VNC_RESULT -ne 0 ]; then
    echo "Failed to start VNC server (exit code: $VNC_RESULT)"
    exit $VNC_RESULT
fi

sleep 2

# Start NoVNC
websockify --daemon --web=/usr/share/novnc/ --cert=/etc/ssl/certs/novnc.crt --key=/etc/ssl/private/novnc.key 6080 localhost:5901

NOVNC_URL="localhost:6080"
# Configure ngrok if NGROK_AUTHTOKEN is set
if [ -n "$NGROK_AUTHTOKEN" ]; then
    ngrok config add-authtoken $NGROK_AUTHTOKEN

    # Start ngrok tunnel for NoVNC
    ngrok http --url=$NGROK_URL 6080 --log=stdout >ngrok.log &

    NOVNC_URL=https://$NGROK_URL
fi

# Print message
echo -e "\033[32m-------------------------------------------------------------------------\033[0m"
echo -e "Note: In order to access VNC and NoVNC on localhost, you need to expose ports 5901 and 6080 to the outside world respectively."
echo -e "\033[32mVNC server is running and accessible at localhost:5901 via VNC viewer\033[0m"
echo -e "\033[32mNoVNC web interface available at $NOVNC_URL/vnc.html via web browser\033[0m"
echo -e "\033[32m-------------------------------------------------------------------------\033[0m"

# Run command
exec "$@"
