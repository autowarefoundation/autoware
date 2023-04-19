SCRIPT_DIR=$(cd $(dirname $0); pwd)
HOME_DIR=$(pwd)

cd $SCRIPT_DIR
cd ../
WORK_SCAPE=$(pwd)
source ./install/setup.bash

ros2 lifecycle set /microstrain_inertial_driver configure

ros2 lifecycle set /microstrain_inertial_driver activate

cd $HOME_DIR