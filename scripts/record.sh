SCRIPT_DIR=$(cd $(dirname $0); pwd)
HOME_DIR=$(pwd)

cd $SCRIPT_DIR
cd ../
WORK_SCAPE=$(pwd)
source ./install/setup.bash
MAP_DIR=$(pwd)/assets

ros2 bag record \
 /sensing/lidar/front/pointcloud_raw \
 /sensing/lidar/rear/pointcloud_raw  \
 /sensing/lidar/front/velodyne_packets \
 /sensing/lidar/rear/velodyne_packets \
 /sensing/camera/c1/camera_info \
 /sensing/camera/c1/image \
 /sensing/camera/c1/image_rect \
 /sensing/camera/default_cam/camera_info \
 /sensing/camera/default_cam/image \
 /sensing/camera/default_cam/image_rect \
 /sensing/imu/imu_data

cd $HOME_DIR