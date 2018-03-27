#!/bin/bash

set -e

ssh -oBatchMode=yes apex-car-drivepx2 '.local/bin/ade enter "source /opt/apex_ws/install.bash; ros2 run lidar_tracking lidar_tracker_exe"'
