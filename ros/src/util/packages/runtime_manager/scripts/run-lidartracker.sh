#!/bin/bash

set -e

exec ssh -tt -oBatchMode=yes 10.31.32.250 '.local/bin/ade enter "sudo bash -c \"export ROS_DOMAIN_ID=250 && source /opt/ros2/local_setup.bash && source /opt/apex_ws/local_setup.bash && lidar_detector_exe --proc_cpu_mask=62 --proc_prio=5\""'
