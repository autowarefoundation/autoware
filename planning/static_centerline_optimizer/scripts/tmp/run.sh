#!/bin/bash

ros2 launch static_centerline_optimizer static_centerline_optimizer.launch.xml run_background:=false lanelet2_input_file_path:="$HOME/AutonomousDrivingScenarios/map/sample_map/lanelet2_map.osm" start_lanelet_id:=125 end_lanelet_id:=132 vehicle_model:=lexus
