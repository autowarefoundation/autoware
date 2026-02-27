#!/usr/bin/env bash
# cspell:ignore openbox, VNC, tigervnc, novnc, websockify, xstartup, pixelformat, vncserver, autoconnect, vncpasswd
# shellcheck disable=SC1090,SC1091
set -e
log_success() { echo -e "\033[32m$*\033[0m"; }
log_info() { echo "$*"; }

# Source ROS/Autoware
source "/opt/ros/$ROS_DISTRO/setup.bash"
source "/opt/autoware/setup.bash"

log_success "--------------------------- Launching Visualizer --------------------------- "

# Openbox config: maximize rviz2 window
mkdir -p /etc/xdg/openbox
cat >/etc/xdg/openbox/rc.xml <<'EOF'
<?xml version="1.0" encoding="UTF-8"?>
<openbox_config xmlns="http://openbox.org/3.4/rc">
  <applications>
    <application name="rviz2">
      <maximized>yes</maximized>
      <focus>yes</focus>
    </application>
  </applications>
</openbox_config>
EOF

# VNC setup
export PASSWORD=${PASSWORD:-openadkit}
mkdir -p ~/.vnc
echo "$PASSWORD" | vncpasswd -f >~/.vnc/passwd && chmod 600 ~/.vnc/passwd
vncserver :99 -geometry 1024x768 -depth 16 -pixelformat rgb565
sleep 1

# Start NoVNC
websockify --daemon --web=/usr/share/novnc/ \
    --cert=/etc/ssl/certs/novnc.crt --key=/etc/ssl/private/novnc.key \
    6080 localhost:5999

# Print access URLs
LOCAL_IP=$(hostname -I | cut -d' ' -f1)
log_success "------------------------------ Visualizer is Ready ------------------------------ "
log_success "PASSWORD: $PASSWORD"
log_success "Local:  http://${LOCAL_IP}:6080/vnc.html?resize=scale&password=${PASSWORD}&autoconnect=true"
if curl -s --head --max-time 2 1.1.1.1 >/dev/null 2>&1; then
    log_success "Public: http://$(curl -s ifconfig.me):6080/vnc.html?resize=scale&password=${PASSWORD}&autoconnect=true"
fi
log_success "---------------------------------------------------------------------------"

export DISPLAY=:99

# Launch rviz2 by default, or execute custom command if provided
if [ $# -eq 0 ]; then
    RVIZ_CONFIG_PATH=${RVIZ_CONFIG_PATH:-$(ros2 pkg prefix --share autoware_launch)/rviz/autoware.rviz}
    USE_SIM_TIME=${USE_SIM_TIME:-false}
    VEHICLE_MODEL=${VEHICLE_MODEL:-sample_vehicle}

    log_success "------------------------------ Launching RViz ------------------------------ "
    log_info "RVIZ_CONFIG_PATH: $RVIZ_CONFIG_PATH"
    log_info "VEHICLE_MODEL: $VEHICLE_MODEL"
    log_info "USE_SIM_TIME: $USE_SIM_TIME"
    log_success "---------------------------------------------------------------------------"

    ros2 launch /autoware/visualizer.launch.xml \
        vehicle_model:="$VEHICLE_MODEL" \
        use_sim_time:="$USE_SIM_TIME" \
        rviz_config_path:="$RVIZ_CONFIG_PATH"
else
    exec "$@"
fi
