#!/usr/bin/env bash

source /etc/profile.d/VimbaGigETL_32bit.sh
source /etc/profile.d/VimbaGigETL_64bit.sh
roslaunch runtime_manager avt_camera.launch "$@"