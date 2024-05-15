#!/bin/bash

ros2 launch autoware_static_centerline_generator static_centerline_generator.launch.xml centerline_source:="bag_ego_trajectory_base" run_background:=false lanelet2_input_file_path:="$HOME/autoware_map/sample_map/lanelet2_map.osm" bag_filename:="$(ros2 pkg prefix autoware_static_centerline_generator --share)/test/data/bag_ego_trajectory.db3" vehicle_model:=lexus
