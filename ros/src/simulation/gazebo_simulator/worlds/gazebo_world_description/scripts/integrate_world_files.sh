#!/bin/sh

SCRIPT_DIR=$(cd $(dirname $0); pwd)
echo "Copy world file to gazebo_world_description"
cp -n $SCRIPT_DIR/../../external/osrf_citysim/worlds/citysim_gazebo7.world $SCRIPT_DIR/../worlds/
cp -n $SCRIPT_DIR/../../external/osrf_citysim/worlds/citysim_gazebo9.world $SCRIPT_DIR/../worlds/
cp -n $SCRIPT_DIR/../../external/car_demo/car_demo/worlds/mcity.world $SCRIPT_DIR/../worlds/
cp -n $SCRIPT_DIR/../../external/car_demo/car_demo/worlds/mcity_r1.world $SCRIPT_DIR/../worlds/
