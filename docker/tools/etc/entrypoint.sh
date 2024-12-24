#!/usr/bin/env bash

configure_vnc() {
    # Create Openbox application configuration
    mkdir -p /etc/xdg/openbox
    cat > /etc/xdg/openbox/rc.xml << 'EOF'
<?xml version="1.0" encoding="UTF-8"?>
<openbox_config xmlns="http://openbox.org/3.4/rc"
                xmlns:xi="http://www.w3.org/2001/XInclude">
  <applications>
    <application name="rviz2">
      <maximized>yes</maximized>
      <position force="yes">
        <x>center</x>
        <y>center</y>
      </position>
      <focus>yes</focus>
      <desktop>1</desktop>
    </application>
  </applications>
</openbox_config>
EOF
    # Create rviz2 start script
    cat > /usr/local/bin/start-rviz2.sh << 'EOF'
#!/bin/bash
source /opt/ros/humble/setup.bash
source /opt/autoware/setup.bash
if [ -n "$RVIZ_CONFIG" ]; then
    exec rviz2 -d "$RVIZ_CONFIG"
else
    exec rviz2
fi
EOF
    chmod +x /usr/local/bin/start-rviz2.sh
    echo "echo 'Autostart executed at $(date)' >> /tmp/autostart.log" >> /etc/xdg/openbox/autostart
    echo "/usr/local/bin/start-rviz2.sh" >> /etc/xdg/openbox/autostart

    # Start VNC server with Openbox
    echo "Starting VNC server with Openbox..."
    vncserver :99 -geometry 1024x768 -depth 16 -pixelformat rgb565
    VNC_RESULT=$?

    if [ $VNC_RESULT -ne 0 ]; then
        echo "Failed to start VNC server (exit code: $VNC_RESULT)"
        exit $VNC_RESULT
    fi

    # Set the DISPLAY variable to match VNC server
    echo "Setting DISPLAY to :99"
    echo "export DISPLAY=:99" >> ~/.bashrc
    sleep 2

    # Start NoVNC
    echo "Starting NoVNC..."
    websockify --daemon --web=/usr/share/novnc/ --cert=/etc/ssl/certs/novnc.crt --key=/etc/ssl/private/novnc.key 6080 localhost:5999

    # Configure ngrok if set
    if [ -n "$NGROK_AUTHTOKEN" ]; then
        ngrok config add-authtoken "$NGROK_AUTHTOKEN"

        if [ -n "$NGROK_URL" ]; then
            ngrok http --url="$NGROK_URL" 6080 --log=stdout >ngrok.log &
        else
            ngrok http 6080 --log=stdout >ngrok.log &
            sleep 2
            NGROK_URL=$(grep -oP 'url=\K[^\s]+' ngrok.log)
        fi
    fi

    # Print info
    echo -e "\033[32m-------------------------------------------------------------------------\033[0m"
    echo -e "\033[32mBrowser interface available at local address http://$(hostname -I | cut -d' ' -f1):6080/vnc.html?resize=scale&password=openadkit&autoconnect=true\033[0m"
    [ -z "$NGROK_AUTHTOKEN" ] && echo -e "\033[32mIf you have a static public ip you can access it on WEB at http://$(curl -s ifconfig.me):6080/vnc.html?resize=scale&password=openadkit&autoconnect=true\033[0m"
    [ -n "$NGROK_AUTHTOKEN" ] && echo -e "\033[32mBrowser interface available at WEB address $NGROK_URL/vnc.html?resize=scale&password=openadkit&autoconnect=true\033[0m"
    echo -e "\033[32m-------------------------------------------------------------------------\033[0m"
}

# shellcheck disable=SC1090
[ "$VNC_ENABLED" == "true" ] && configure_vnc
source "/opt/ros/$ROS_DISTRO/setup.bash"
source "/opt/autoware/setup.bash"
exec "$@"