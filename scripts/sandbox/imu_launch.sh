SCRIPT_DIR=$(cd $(dirname $0); pwd)
HOME_DIR=$(pwd)

cd $SCRIPT_DIR
cd ../
WORK_SCAPE=$(pwd)
source ./install/setup.bash
MAP_DIR=$(pwd)/assets

ros2 launch logiee-s-tc_sensor_kit_launch imu.launch.xml vehicle_model:=logiee-s-tc sensor_model:=logiee-s-tc_sensor_kit map_path:=/PATH/TO/YOUR/MAP

cd $HOME_DIR