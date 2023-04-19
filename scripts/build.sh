SCRIPT_DIR=$(cd $(dirname $0); pwd)
HOME_DIR=$(pwd)

cd $SCRIPT_DIR
cd ../
WORK_SCAPE=$(pwd)
source ./install/setup.bash
MAP_DIR=$(pwd)/assets

# copy calib file 
cp ./src/sensor_kit/logiee-s-tc_sensor_kit_launch/logiee-s-tc_sensor_kit_description/config/sensors_calibration.yaml \
   ./src/param/autoware_individual_params/individual_params/config/default/logiee-s-tc_sensor_kit/sensors_calibration.yaml

cp ./src/sensor_kit/logiee-s-tc_sensor_kit_launch/logiee-s-tc_sensor_kit_description/config/sensor_kit_calibration.yaml \
   ./src/param/autoware_individual_params/individual_params/config/default/logiee-s-tc_sensor_kit/sensor_kit_calibration.yaml

colcon build --packages-select \
 logiee-s-tc_sensor_kit_launch \
 logiee-s-tc_description \
 logiee-s-tc_sensor_kit_description \
 logiee-s-tc_sensor_kit_launch_description \
 logiee-s-tc_launch \
 logiee-s-tc_sensor_kit_launch \
 microstrain_inertial \
 individual_params \
 microstrain_inertial_driver \
 jw_interface_awiv_adapter \
 jw_interface_msgs \
 jw_interface_serial \
 common_sensor_launch \
 extrinsic_calibration_client \
 extrinsic_calibration_manager \
 extrinsic_dummy_calibrator \
 extrinsic_ground_plane_calibrator \
 extrinsic_interactive_calibrator \
 extrinsic_lidar_to_lidar_2d_calibrator \
 extrinsic_manual_calibrator \
 extrinsic_map_based_calibrator \
 extrinsic_tag_based_calibrator \
 intrinsic_camera_calibration \
 intrinsic_camera_calibrator \
 point_cloud_accumulator 


cd $HOME_DIR