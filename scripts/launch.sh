SCRIPT_DIR=$(cd $(dirname $0); pwd)
HOME_DIR=$(pwd)

cd $SCRIPT_DIR

ros2 launch autoware_launch autoware.launch.xml vehicle_model:=logiee-s-tc sensor_kit:=logiee-s-tc_sensor_kit_launch map_path:=/PATH/TO/YOUR/MAP

cd $HOME_DIR