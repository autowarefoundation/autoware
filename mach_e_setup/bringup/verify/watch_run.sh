#!/usr/bin/env bash
# Wait for a new run dir from run_mach_e.sh, start a planning/control bag, then stream deduped errors + GPU mem.
RUNS=$HOME/autoware_logs/runs; S=$HOME/utils/mach_e_bringup/verify
before=$(ls -1 $RUNS | sort | tail -1); latest=$before
while [ "$latest" = "$before" ]; do sleep 2; latest=$(ls -1 $RUNS | sort | tail -1); done
D=$RUNS/$latest; echo "NEW RUN: $D"
until [ -f $D/autoware.log ]; do sleep 1; done
( source $HOME/utils/autoware_env_setup/aw.sh >/dev/null 2>&1
  sleep 60; OUT=/mnt/mapdata/autoware_runs/${latest}_planning; echo $OUT > $S/bagpath
  ros2 bag record -s mcap -o $OUT /planning/planning_factors/stop_line /planning/planning_factors/traffic_light /api/planning/velocity_factors /planning/trajectory /planning/scenario_planning/lane_driving/behavior_planning/path /planning/mission_planning/route /localization/kinematic_state /localization/acceleration /control/command/control_cmd /control/trajectory_follower/control_cmd /api/operation_mode/state /system/fail_safe/mrm_state /system/operation_mode/availability /vehicle/status/control_mode /vehicle/status/velocity_status /vehicle/status/steering_status /vehicle/brake/report /vehicle/throttle/report /vehicle/ulc/report /vehicle/system/report /control/command/emergency_cmd /diagnostics_graph/status /diagnostics_graph/struct /tf_static > $S/bag.log 2>&1 ) &
( i=0; while sleep 30; do i=$((i+1)); m=$(nvidia-smi --query-gpu=memory.used --format=csv,noheader,nounits); rp=$(pgrep -f "^/opt/ros/humble/lib/rviz2/rviz2 -d" | head -1); rv=$(nvidia-smi | awk -v p=$rp '$5==p{gsub("MiB","",$(NF-1));print $(NF-1)}'); [ $((i % 10)) -eq 0 ] && echo "GPU status: total ${m} MiB, rviz ${rv:-?} MiB"; [ "$m" -gt 12000 ] && echo "GPU MEM HIGH: ${m} MiB (rviz ${rv:-?} MiB)"; done ) &
tail -n +1 -F $D/autoware.log $D/adma.log $D/display_lidar.log $D/display_map.log 2>/dev/null | grep --line-buffered -E "\[(ERROR|FATAL)\]|process has died|Traceback|exception|Segmentation|terminate called|Killed|out of memory|bad_alloc|No stop point found" | grep --line-buffered -vE "VehicleCmdFilter|not stopped" | \
awk '{ k=$0; gsub(/[0-9]+(\.[0-9]+)?/,"#",k); gsub(/\[[^]]*\] /,"",k); c[k]++; n=c[k];
  if (n==1 || n==10 || n==100 || n==1000) { print (n>1 ? "[x" n "] " : "") substr($0,1,350); fflush() } }'
