#!/usr/bin/env bash
# cspell:ignore openbox, xstartup, VNC, novnc, websockify, pixelformat, vncserver, autoconnect, vncpasswd
# shellcheck disable=SC1090,SC1091

configure_vnc() {
    # Configure VNC password
    if [ -z "$PASSWORD" ]; then
        echo -e "\e[31mPASSWORD is not set, using *openadkit* as default\e[0m"
        PASSWORD="openadkit"
    fi
    mkdir -p ~/.vnc
    echo "$PASSWORD" | vncpasswd -f >~/.vnc/passwd && chmod 600 ~/.vnc/passwd

    # Create X startup script
    cat >~/.vnc/xstartup <<'EOF'
#!/bin/bash
export PATH="/usr/bin:/usr/local/bin:$PATH"
unset SESSION_MANAGER
# Ensure a D-Bus session is running for Openbox
dbus-launch --exit-with-session /usr/bin/openbox-session
if [ $? -ne 0 ]; then
    echo "Error: openbox-session failed to start." >&2 # Log to stderr
    # Optionally, log to a persistent file if needed for post-mortem
    # echo "openbox-session failed at $(date)" >> "$HOME/.vnc/xstartup_errors.log"
fi
EOF
    chmod +x ~/.vnc/xstartup

    # Create Openbox application configuration
    mkdir -p /etc/xdg/openbox
    cat >/etc/xdg/openbox/rc.xml <<EOF
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

    # Start VNC server
    echo "Starting VNC server (DISPLAY=$DISPLAY)..."
    DISPLAY_NUMBER=${DISPLAY#:}
    rm -f /tmp/.X"${DISPLAY_NUMBER}"-lock /tmp/.X11-unix/X"${DISPLAY_NUMBER}" # Clean up potential stale lock files

    vncserver "$DISPLAY" -geometry 1024x768 -depth 16 -pixelformat rgb565 -localhost no -verbose
    VNC_RESULT=$?

    if [ $VNC_RESULT -ne 0 ]; then
        echo "Error: Failed to start VNC server (exit code: $VNC_RESULT)"
        RFB_PORT_FOR_LOG=$((5900 + DISPLAY_NUMBER))
        VNC_LOG_FILE="$HOME/.vnc/$(hostname):${RFB_PORT_FOR_LOG}.log"
        if [ -f "$VNC_LOG_FILE" ]; then
            echo "VNC server log ($VNC_LOG_FILE) contents:"
            cat "$VNC_LOG_FILE"
        else
            echo "VNC server log ($VNC_LOG_FILE) not found. Listing $HOME/.vnc/ contents:"
            ls -la "$HOME/.vnc/"
        fi
        exit $VNC_RESULT
    fi

    echo "VNC server started successfully."
    sleep 1 # Brief pause for VNC server to initialize fully

    # Start NoVNC
    echo "Starting NoVNC..."
    RFB_PORT=$((5900 + DISPLAY_NUMBER))
    websockify --daemon --web=/usr/share/novnc/ --cert=/etc/ssl/certs/novnc.crt --key=/etc/ssl/private/novnc.key 6080 localhost:${RFB_PORT}

    # Print connection info
    echo -e "\033[32m-------------------------------------------------------------------------\033[0m"
    echo -e "\033[32mBrowser interface available at http://localhost:6080/vnc.html?resize=scale&password=${PASSWORD}&autoconnect=true\033[0m"
    # Attempt to print container IP for convenience, if 'hostname -I' is available and works
    CONTAINER_IP=$(hostname -I | awk '{print $1}' 2>/dev/null)
    if [ -n "$CONTAINER_IP" ]; then
        echo -e "\033[32mBrowser interface also at http://${CONTAINER_IP}:6080/vnc.html?resize=scale&password=${PASSWORD}&autoconnect=true\033[0m"
    fi
    if curl -s --head --connect-timeout 2 1.1.1.1 >/dev/null 2>&1; then # Check internet with timeout
        PUBLIC_IP=$(curl -s --connect-timeout 2 ifconfig.me 2>/dev/null)
        if [ -n "$PUBLIC_IP" ]; then
            echo -e "\033[32mIf publicly accessible, try http://${PUBLIC_IP}:6080/vnc.html?resize=scale&password=${PASSWORD}&autoconnect=true\033[0m"
        fi
    else
        echo -e "\033[33mNo internet connection detected or ifconfig.me unreachable.\033[0m"
    fi
    echo -e "\033[32m-------------------------------------------------------------------------\033[0m"
}

configure_vnc

echo "Visualizer entrypoint finished configuration. Keeping container alive with sleep infinity..."
sleep infinity
