#!/bin/sh

usage_exit() {
    echo "Usage: ros2 run autoware_rosbag_recorder record.sh [-o filename]" 1>&2
    exit 1
}

while getopts o:h OPT; do
    case $OPT in
    "o") echo "record as $OPTARG" ;;
    "h") usage_exit ;;
    "*") usage_exit ;;
    \?) usage_exit ;;
    esac
done

if [ -n "$OPTARG" ]; then
    ros2 bag record -e "(.*)/velodyne_packets|/pacmod_interface/(.*)|/pacmod/(.*)|/vehicle/(.*)|/sensing/imu/(.*)|/sensing/gnss/(.*)|/sensing/camera/(.*)/camera_info|/sensing/camera/(.*)/compressed|/perception/object_recognition/detection/rois(.)|/perception/object_recognition/objects" -o "$OPTARG"
else
    usage_exit
fi
