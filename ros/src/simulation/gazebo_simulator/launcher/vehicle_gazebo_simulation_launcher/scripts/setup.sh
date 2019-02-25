#!/bin/sh

SCRIPT_DIR=$(cd $(dirname $0); pwd)
$SCRIPT_DIR/download_model.sh
$SCRIPT_DIR/upgrade_gazebo_stable_version.sh
$SCRIPT_DIR/integrate_world_files.sh