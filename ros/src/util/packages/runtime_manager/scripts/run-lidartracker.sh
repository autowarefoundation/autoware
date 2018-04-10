#!/bin/bash

set -e

ssh -t -oBatchMode=yes 10.31.32.250 '.local/bin/ade enter "sudo bash -c \"source /opt/ros2/local_setup.bash && source /opt/apex_ws/local_setup.bash && lidar_detector_exe --proc_cpu_mask=62 --proc_prio=5\""'
