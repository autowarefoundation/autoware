SCRIPT_DIR=$(cd $(dirname $0); pwd)
HOME_DIR=$(pwd)

cd $SCRIPT_DIR
cd ../
WORK_SCAPE=$(pwd)
source ./install/setup.bash
MAP_DIR=$(pwd)/assets

ros2 bag record /sensing/lidar/front/pointcloud_raw /sensing/lidar/rear/pointcloud_raw  /sensing/lidar/front/velodyne_packets  /sensing/lidar/rear/velodyne_packets


cd $HOME_DIR