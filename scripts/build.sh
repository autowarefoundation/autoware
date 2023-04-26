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
 logiee-s-tc_launch \
 individual_params \
 jw_interface_awiv_adapter \
 jw_interface_msgs \
 jw_interface_serial \
 common_sensor_launch \
 autoware_launch \
 tier4_localization_launch \
 point_cloud_accumulator \
 vehicle_info_util \
 pointcloud_preprocessor \
 microstrain_inertial_driver \
 v4l2_camera

#  ndt_scan_matcher \
#  static_centerline_optimizer \

cd $HOME_DIR