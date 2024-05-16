#!/bin/bash

# planning simulation
function download_planning_map() {
    if [ ! -f ~/autoware_map/sample-map-planning.zip ]; then
        gdown -O ~/autoware_map/ 'https://docs.google.com/uc?export=download&id=1499_nsbUbIeturZaDj7jhUownh5fvXHd'
        unzip -d ~/autoware_map ~/autoware_map/sample-map-planning.zip
    fi
}
alias awf-launch-planning-sim='download_planning_map&&ros2 launch autoware_launch planning_simulator.launch.xml map_path:=$HOME/autoware_map/sample-map-planning vehicle_model:=sample_vehicle sensor_model:=sample_sensor_kit'

# rosbag replay simulation
function download_rosbag_map() {
    if [ ! -f ~/autoware_map/sample-map-rosbag.zip ]; then
        gdown -O ~/autoware_map/ 'https://docs.google.com/uc?export=download&id=1A-8BvYRX3DhSzkAnOcGWFw5T30xTlwZI'
        unzip -d ~/autoware_map/ ~/autoware_map/sample-map-rosbag.zip
    fi
}
alias awf-launch-sample-rosbag-sim='download_rosbag_artifacts&&download_rosbag_map&&ros2 launch autoware_launch logging_simulator.launch.xml map_path:=$HOME/autoware_map/sample-map-rosbag vehicle_model:=sample_vehicle sensor_model:=sample_sensor_kit'

# play sample rosbag
function download_rosbag_file() {
    if [ ! -f ~/autoware_map/sample-rosbag.zip ]; then
        gdown -O ~/autoware_map/ 'https://docs.google.com/uc?export=download&id=1VnwJx9tI3kI_cTLzP61ktuAJ1ChgygpG'
        unzip -d ~/autoware_map/ ~/autoware_map/sample-rosbag.zip
    fi
}
alias awf-play-sample-rosbag='download_rosbag_file&&ros2 bag play ~/autoware_map/sample-rosbag/sample.db3 -r 0.35 -s sqlite3'
