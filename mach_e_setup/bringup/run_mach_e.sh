#!/usr/bin/env bash
# Bring up Autoware on the real Mach-E: the ADMA GNSS/INS driver + autoware.launch.xml
# (Ouster lidar, LUCID camera, DBW2 vehicle interface, localization, perception, planning, control).
#
#   run_mach_e.sh                      # map PD2LAB, RViz on
#   run_mach_e.sh USF_map              # another map under ~/autoware_map
#   run_mach_e.sh PD2LAB rviz:=false   # anything after the map is passed to ros2 launch
#
# Env overrides:
#   ADMA_CONFIG=/path/driver_config.yaml   a different ADMA (e.g. the bus); default is the package config
#   ADMA_LOG_GSDB=True                     also write the ADMA raw .gsdb log into the run directory
#   RVIZ_LIGHT=1                           display/autoware_light.rviz (voxel map, throttled clouds) instead of stock
#   RVIZ_LIDAR_POINTS / RVIZ_CLOUD_RATE / RVIZ_OBSTACLE_POINTS / RVIZ_MAP_VOXEL   tune RVIZ_LIGHT (see display/)
#   RVIZ_GPU_LIMIT=13000                   total GPU MiB at which RViz is restarted (rviz2 leak safety net)
#
# Nothing moves until someone engages AUTONOMOUS (RViz "Engage" / AD API). Until then the
# DBW2 vehicle interface publishes /vehicle/disable every cycle and sends no motion commands.
set -eo pipefail

MAP=${1:-PD2LAB}; shift || true
MAP_PATH=$HOME/autoware_map/$MAP
[ -f "$MAP_PATH/lanelet2_map.osm" ] || { echo "no lanelet2_map.osm in $MAP_PATH"; exit 1; }

source "$HOME/utils/autoware_env_setup/aw.sh"

RUN_DIR=$HOME/autoware_logs/runs/$(date +%Y-%m-%d_%H%M%S)_$MAP
mkdir -p "$RUN_DIR"
export ROS_LOG_DIR=$RUN_DIR/ros_log
echo "map: $MAP_PATH"
echo "logs: $RUN_DIR"

ADMA_ARGS=(log_gsdb:=${ADMA_LOG_GSDB:-False} rosbag_path:=$RUN_DIR)
[ -n "${ADMA_CONFIG:-}" ] && ADMA_ARGS+=(driver_config:=$ADMA_CONFIG)
ros2 launch adma_ros2_driver adma_driver.launch.py "${ADMA_ARGS[@]}" > "$RUN_DIR/adma.log" 2>&1 &
ADMA_PID=$!
PIDS=($ADMA_PID)
trap 'kill -INT "${PIDS[@]}" 2>/dev/null; wait "${PIDS[@]}" 2>/dev/null' EXIT

# RViz GPU leak (2026-10-15): the NVIDIA driver's automatic GL threaded optimizations made rviz2 leak
# every pointcloud vertex buffer (~3 GB/min with the full lidar; the GPU filled and the CUDA pointcloud
# preprocessor died). Fixed by ~/.nv/nvidia-application-profiles-rc (GLThreadedOptimizations=false for
# rviz2). RVIZ_LIGHT=1 keeps the old workaround: voxel map published once + throttled clouds.
RVIZ_ARGS=()
if [ -n "${RVIZ_LIGHT:-}" ]; then
  DISPLAY_DIR=$HOME/utils/mach_e_bringup/display
  python3 "$DISPLAY_DIR/display_map_pub.py" "$MAP_PATH" "${RVIZ_MAP_VOXEL:-0.5}" > "$RUN_DIR/display_map.log" 2>&1 &
  PIDS+=($!)
  python3 "$DISPLAY_DIR/display_cloud_throttle.py" --rate "${RVIZ_CLOUD_RATE:-2}" --points "${RVIZ_LIDAR_POINTS:-20000}" --obstacle-points "${RVIZ_OBSTACLE_POINTS:-50000}" \
    > "$RUN_DIR/display_lidar.log" 2>&1 &
  PIDS+=($!)
  RVIZ_ARGS=(rviz_config:=$DISPLAY_DIR/autoware_light.rviz)
fi
# Backstop: restart RViz if the GPU still fills up (e.g. the NVIDIA profile is missing), before it
# starves the lidar pipeline.
"$HOME/utils/mach_e_bringup/display/rviz_gpu_guard.sh" "${RVIZ_GPU_LIMIT:-13000}" > "$RUN_DIR/rviz_gpu_guard.log" 2>&1 &
PIDS+=($!)

ros2 launch autoware_launch autoware.launch.xml \
  map_path:="$MAP_PATH" \
  vehicle_model:=mach_e_vehicle \
  sensor_model:=mach_e_sensor_kit \
  "${RVIZ_ARGS[@]}" "$@" 2>&1 | tee "$RUN_DIR/autoware.log"
