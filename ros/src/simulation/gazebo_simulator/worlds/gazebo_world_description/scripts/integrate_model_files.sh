#!/bin/sh

SCRIPT_DIR=$(cd $(dirname $0); pwd)
echo "Copy world file to gazebo_world_description"
cp -rn $SCRIPT_DIR/../../external/osrf_citysim/models/actor $SCRIPT_DIR/../models/
cp -rn $SCRIPT_DIR/../../external/osrf_citysim/models/city_terrain $SCRIPT_DIR/../models/
cp -rn $SCRIPT_DIR/../../external/osrf_citysim/models/ocean $SCRIPT_DIR/../models/
cp -rn $SCRIPT_DIR/../../external/car_demo/car_demo/models/* $SCRIPT_DIR/../models/
