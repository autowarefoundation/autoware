#!/usr/bin/env bash
# cspell:ignore openbox, VNC, tigervnc, novnc, websockify, newkey, xstartup, pixelformat, AUTHTOKEN, authtoken, vncserver, autoconnect, vncpasswd
# shellcheck disable=SC1090,SC1091

# Check if RVIZ_CONFIG is provided
if [ -z "$RVIZ_CONFIG" ]; then
    echo -e "\e[31mRVIZ_CONFIG is not set defaulting to /autoware/rviz/autoware.rviz\e[0m"
    RVIZ_CONFIG="/autoware/rviz/autoware.rviz"
    export RVIZ_CONFIG
fi

configure_vnc() {
    # Create Openbox application configuration
    mkdir -p /etc/xdg/openbox
    cat >/etc/xdg/openbox/rc.xml <<'EOF'
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
    cat >/usr/local/bin/start-rviz2.sh <<'EOF'
#!/bin/bash
source /opt/ros/"$ROS_DISTRO"/setup.bash
source /opt/autoware/setup.bash
exec rviz2 -d "$RVIZ_CONFIG"
EOF
    chmod +x /usr/local/bin/start-rviz2.sh
    echo "echo 'Autostart executed at $(date)' >> /tmp/autostart.log" >>/etc/xdg/openbox/autostart
    echo "/usr/local/bin/start-rviz2.sh" >>/etc/xdg/openbox/autostart

    # Configure VNC password
    if [ -z "$REMOTE_PASSWORD" ]; then
        echo -e "\e[31mREMOTE_PASSWORD is not set, using *openadkit* as default\e[0m"
        REMOTE_PASSWORD="openadkit"
    fi
    mkdir -p ~/.vnc
    echo "$REMOTE_PASSWORD" | vncpasswd -f >~/.vnc/passwd && chmod 600 ~/.vnc/passwd

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
    echo "export DISPLAY=:99" >>~/.bashrc
    sleep 2

    # Start NoVNC
    echo "Starting NoVNC..."
    websockify --daemon --web=/usr/share/novnc/ --cert=/etc/ssl/certs/novnc.crt --key=/etc/ssl/private/novnc.key 6080 localhost:5999

    # Print info
    echo -e "\033[32m-------------------------------------------------------------------------\033[0m"
    echo -e "\033[32mBrowser interface available at local address http://$(hostname -I | cut -d' ' -f1):6080/vnc.html?resize=scale&password=${REMOTE_PASSWORD}&autoconnect=true\033[0m"
    if curl -s --head 1.1.1.1 >/dev/null 2>&1; then
        echo -e "\033[32mIf you have a static public ip you can access it on WEB at http://$(curl -s ifconfig.me):6080/vnc.html?resize=scale&password=${REMOTE_PASSWORD}&autoconnect=true\033[0m"
    else
        echo -e "\033[31mNo internet connection available\033[0m"
    fi
    echo -e "\033[32m-------------------------------------------------------------------------\033[0m"
}

# Source ROS and Autoware setup files
source "/opt/ros/$ROS_DISTRO/setup.bash"
source "/opt/autoware/setup.bash"

# Execute passed command if provided, otherwise launch rviz2
if [ "$REMOTE_DISPLAY" == "false" ]; then
    [ $# -eq 0 ] && rviz2 -d "$RVIZ_CONFIG"
    exec "$@"
else
    configure_vnc
    [ $# -eq 0 ] && sleep infinity
    exec "$@"
fi
