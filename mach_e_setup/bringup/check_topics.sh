#!/usr/bin/env bash
# With run_mach_e.sh running, print the rate of each sensor, vehicle and localization topic.
# Run in a second terminal: ~/utils/mach_e_bringup/check_topics.sh
source "$HOME/utils/autoware_env_setup/aw.sh"
TOPICS=(
  /sensing/lidar/top/pointcloud_raw_ex          # Ouster via Nebula
  /sensing/lidar/concatenated/pointcloud        # after preprocessing
  /sensing/camera/camera0/image_raw             # LUCID
  /sensing/camera/camera0/camera_info
  /genesys/adma/fix                             # ADMA GNSS
  /genesys/adma/imu                             # ADMA IMU
  /sensing/imu/imu_data
  /sensing/gnss/pose_with_covariance
  /vehicle/status/velocity_status               # DBW2 via ds_dbw_autoware
  /vehicle/status/steering_status
  /vehicle/status/control_mode
  /localization/kinematic_state
  /perception/object_recognition/objects
)
for t in "${TOPICS[@]}"; do
  r=$(timeout 6 ros2 topic hz "$t" 2>/dev/null | awk '/average rate/{print $3; exit}')
  printf '%-48s %s\n' "$t" "${r:-NO DATA} ${r:+Hz}"
done
