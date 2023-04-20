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
 /sensing/lidar/rear/velodyne_packets \
 /sensing/lidar/rear/velodyne_packets \
 /sensing/lidar/rear/velodyne_packets \
 /sensing/camera/c1_cam/camera_info \
 /sensing/camera/c1_cam/image_raw \
 /sensing/camera/c1_cam/image_raw/compressed \
 /sensing/camera/c1_cam/image_raw/compressedDepth \
 /sensing/camera/c1_cam/image_raw/theora \
 /sensing/camera/default_cam/camera_info \
 /sensing/camera/default_cam/image_raw \
 /sensing/camera/default_cam/image_raw/compressed \
 /sensing/camera/default_cam/image_raw/compressedDepth \
 /sensing/camera/default_cam/image_raw/theora \
 /sensing/camera/traffic_light/camera_info \
 /sensing/camera/traffic_light/image_raw \
 /sensing/camera/traffic_light/image_raw/compressed

cd $HOME_DIR