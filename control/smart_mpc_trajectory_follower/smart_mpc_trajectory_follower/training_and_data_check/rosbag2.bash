#!/bin/bash

gnome-terminal -- bash -c 'ros2 topic echo /localization/kinematic_state nav_msgs/msg/Odometry --csv --qos-history keep_all --qos-reliability reliable > kinematic_state.csv'
gnome-terminal -- bash -c 'ros2 topic echo /localization/acceleration geometry_msgs/msg/AccelWithCovarianceStamped --csv --qos-history keep_all --qos-reliability reliable > acceleration.csv'
gnome-terminal -- bash -c 'ros2 topic echo /vehicle/status/steering_status autoware_vehicle_msgs/msg/SteeringReport --csv --qos-history keep_all --qos-reliability reliable > steering_status.csv'

gnome-terminal -- bash -c 'ros2 topic echo /control/command/control_cmd autoware_control_msgs/msg/Control --csv --qos-history keep_all --qos-reliability reliable > control_cmd.csv'
gnome-terminal -- bash -c 'ros2 topic echo /control/trajectory_follower/control_cmd autoware_control_msgs/msg/Control --csv --qos-history keep_all --qos-reliability reliable > control_cmd_orig.csv'

gnome-terminal -- bash -c 'ros2 topic echo /control/trajectory_follower/lane_departure_checker_node/debug/deviation/lateral tier4_debug_msgs/msg/Float64Stamped --csv --qos-history keep_all --qos-reliability reliable > lateral_error.csv'
gnome-terminal -- bash -c 'ros2 topic echo /control/trajectory_follower/lane_departure_checker_node/debug/deviation/yaw tier4_debug_msgs/msg/Float64Stamped --csv --qos-history keep_all --qos-reliability reliable > yaw_error.csv'

gnome-terminal -- bash -c 'ros2 topic echo /system/operation_mode/state autoware_adapi_v1_msgs/msg/OperationModeState --csv --qos-history keep_all --qos-reliability reliable > system_operation_mode_state.csv'
gnome-terminal -- bash -c 'ros2 topic echo /vehicle/status/control_mode autoware_vehicle_msgs/msg/ControlModeReport --csv --qos-history keep_all --qos-reliability reliable > vehicle_status_control_mode.csv'
gnome-terminal -- bash -c 'ros2 topic echo /sensing/imu/imu_data sensor_msgs/msg/Imu --csv --qos-history keep_all --qos-reliability reliable > imu.csv'

gnome-terminal -- bash -c 'ros2 topic echo /debug_mpc_x_des tier4_debug_msgs/msg/Float32MultiArrayStamped --csv --qos-history keep_all --qos-reliability reliable > debug_mpc_x_des.csv'
gnome-terminal -- bash -c 'ros2 topic echo /debug_mpc_y_des tier4_debug_msgs/msg/Float32MultiArrayStamped --csv --qos-history keep_all --qos-reliability reliable > debug_mpc_y_des.csv'
gnome-terminal -- bash -c 'ros2 topic echo /debug_mpc_v_des tier4_debug_msgs/msg/Float32MultiArrayStamped --csv --qos-history keep_all --qos-reliability reliable > debug_mpc_v_des.csv'
gnome-terminal -- bash -c 'ros2 topic echo /debug_mpc_yaw_des tier4_debug_msgs/msg/Float32MultiArrayStamped --csv --qos-history keep_all --qos-reliability reliable > debug_mpc_yaw_des.csv'
gnome-terminal -- bash -c 'ros2 topic echo /debug_mpc_acc_des tier4_debug_msgs/msg/Float32MultiArrayStamped --csv --qos-history keep_all --qos-reliability reliable > debug_mpc_acc_des.csv'
gnome-terminal -- bash -c 'ros2 topic echo /debug_mpc_steer_des tier4_debug_msgs/msg/Float32MultiArrayStamped --csv --qos-history keep_all --qos-reliability reliable > debug_mpc_steer_des.csv'
gnome-terminal -- bash -c 'ros2 topic echo /debug_mpc_X_des_converted tier4_debug_msgs/msg/Float32MultiArrayStamped --csv --qos-history keep_all --qos-reliability reliable > debug_mpc_X_des_converted.csv'
gnome-terminal -- bash -c 'ros2 topic echo /debug_mpc_x_current tier4_debug_msgs/msg/Float32MultiArrayStamped --csv --qos-history keep_all --qos-reliability reliable > debug_mpc_x_current.csv'
gnome-terminal -- bash -c 'ros2 topic echo /debug_mpc_error_prediction tier4_debug_msgs/msg/Float32MultiArrayStamped --csv --qos-history keep_all --qos-reliability reliable > debug_mpc_error_prediction.csv'
gnome-terminal -- bash -c 'ros2 topic echo /debug_mpc_max_trajectory_err tier4_debug_msgs/msg/Float32Stamped --csv --qos-history keep_all --qos-reliability reliable > debug_mpc_max_trajectory_err.csv'
gnome-terminal -- bash -c 'ros2 topic echo /debug_mpc_emergency_stop_mode tier4_debug_msgs/msg/BoolStamped --csv --qos-history keep_all --qos-reliability reliable > debug_mpc_emergency_stop_mode.csv'
gnome-terminal -- bash -c 'ros2 topic echo /debug_mpc_goal_stop_mode tier4_debug_msgs/msg/BoolStamped --csv --qos-history keep_all --qos-reliability reliable > debug_mpc_goal_stop_mode.csv'
gnome-terminal -- bash -c 'ros2 topic echo /debug_mpc_total_ctrl_time tier4_debug_msgs/msg/Float32Stamped --csv --qos-history keep_all --qos-reliability reliable > debug_mpc_total_ctrl_time.csv'
gnome-terminal -- bash -c 'ros2 topic echo /debug_mpc_calc_u_opt_time tier4_debug_msgs/msg/Float32Stamped --csv --qos-history keep_all --qos-reliability reliable > debug_mpc_calc_u_opt_time.csv'

# wait a moment to open new terminals converting rosbag2 to csv
sleep 8

ros2 bag play rosbag2_*.db3 -r 10

sleep 3

# kill terminals converting rosbag2 to csv
pgrep -f "\ --csv\ --qos-history\ keep_all\ --qos-reliability\ reliable" | xargs kill -9
