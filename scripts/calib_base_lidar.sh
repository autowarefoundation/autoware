SCRIPT_DIR=$(cd $(dirname $0); pwd)
HOME_DIR=$(pwd)

cd $SCRIPT_DIR
cd ../
WORK_SCAPE=$(pwd)
source ./install/setup.bash
MAP_DIR=$(pwd)/assets

ros2 launch extrinsic_calibration_manager calibration.launch.xml \
  mode:=ground_plane sensor_model:=logiee-s-tc_sensor_kit vehicle_model:=logiee-s-tc vehicle_id:=default \
  # logging_simulator:=false

cd $HOME_DIR