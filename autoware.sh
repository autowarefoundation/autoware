#!/bin/bash
# get where I am
MY_PATH=$(readlink -f  $(dirname $0))

# boot ros-master
gnome-terminal --geometry=50x10+0+0 --title="roscore" --working-directory=${MY_PATH} --command="bash -c 'source ./ros/devel/setup.bash; roscore'"&

# boot runtime_manager
gnome-terminal --geometry=50x10+500+0 --title="runtime_manager" --working-directory=${MY_PATH} --command="bash -c 'source ./ros/devel/setup.bash; rosrun runtime_manager runtime_manager.py'"
