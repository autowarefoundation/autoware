#!/bin/bash

declare -A node_name_dict
declare -A logger_name_dict

# behavior path planner
behavior_path_module_list=("avoidance" "avoidance_by_lane_change" "dynamic_avoidance" "lane_change_right" "lane_change_left" "external_request_lane_change_right" "external_request_lane_change_left" "goal_planner" "start_planner" "side_shift")
for module in "${behavior_path_module_list[@]}"; do
    node_name_dict[$module]="/planning/scenario_planning/lane_driving/behavior_planning/behavior_path_planner"
    logger_name_dict[$module]="planning.scenario_planning.lane_driving.behavior_planning.behavior_path_planner."$module
done

# behavior velocity planner
behavior_velocity_module_list=("crosswalk" "walkway" "traffic_light" "intersection" "merge_from_private" "blind_spot" "detection_area" "virtual_traffic_light" "no_stopping_area" "stop_line" "occlusion_spot" "run_out" "speed_bump" "out_of_lane" "no_drivable_lane")
for module in "${behavior_velocity_module_list[@]}"; do
    node_name_dict[$module]="/planning/scenario_planning/lane_driving/behavior_planning/behavior_velocity_planner"
    logger_name_dict[$module]="planning.scenario_planning.lane_driving.behavior_planning.behavior_velocity_planner."$module
done

# obstacle avoidance planner
node_name_dict["obstacle_avoidance_planner"]=/planning/scenario_planning/lane_driving/motion_planning/obstacle_avoidance_planner
logger_name_dict["obstacle_avoidance_planner"]=/planning/scenario_planning/lane_driving/motion_planning/obstacle_avoidance_planner

# motion velocity smoother
node_name_dict["motion_velocity_smoother"]=/planning/scenario_planning/motion_velocity_smoother
logger_name_dict["motion_velocity_smoother"]=/planning/scenario_planning/motion_velocity_smoother

if [ -z "${node_name_dict[$1]}" ]; then
    echo "[ERROR] $1 is not found."
    echo -n "[ERROR] The available modules are [ "
    for node_name in "${!node_name_dict[@]}"; do
        echo -n "${node_name} "
    done
    echo "]"
    exit 0
fi

# update logger
node_name=${node_name_dict[$1]}
logger_name=${logger_name_dict[$1]}
ros2 service call "$node_name/config_logger" logging_demo/srv/ConfigLogger "{logger_name: $logger_name, level: $2}"
