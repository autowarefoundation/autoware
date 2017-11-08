#!/usr/bin/env python
# coding: utf-8

from subprocess import call
from os import getcwd, chdir

rootpath = getcwd()
print rootpath

# pip
#call(["pip", "install", "--upgrade", "pip"])
call(["sudo", "pip", "install", "-r", "requirements.txt"])
call(["sudo", "pip", "install", "git+https://github.com/strawlab/python-pcl"])

# ROSBridge
call(["sudo", "apt-get", "update"])
call(["sudo", "apt-get", "install", "ros-kinetic-rosbridge-server"])
call(["sudo", "apt-get", "install", "ros-kinetic-tf2-web-republisher"])
call(["sudo", "apt-get", "install", "ros-kinetic-web-video-server"])

# npm, node
#call(["sudo", "apt-get", "install", "-y", "nodejs", "npm"])
#call(["sudo", "npm", "cache", "clean"])
#call(["sudo", "npm", "install", "n", "-g"])
#call(["sudo", "n", "stable"])
#call(["sudo", "ln", "-sf", "/usr/local/bin/node", "/usr/bin/node"])

chdir("{}/app/views".format(rootpath))
call(["npm", "install"])
call(["sudo", "npm", "install", "--save", "-g", "gulp-cli"])

# res
chdir("{}/app/controllers/res/detection".format(rootpath))
call(["wget", "https://autoware.blob.core.windows.net/web-ui-resouces/calibration_camera_lidar_3d_prius_nic-150407.yml"])

chdir("{}/app/controllers/res/map".format(rootpath))
call(["wget", "https://autoware.blob.core.windows.net/web-ui-resouces/points.tar.gz"])
call(["tar", "-zxvf", "points.tar.gz"])
call(["wget", "https://autoware.blob.core.windows.net/web-ui-resouces/vectors.tar.gz"])
call(["tar", "-zxvf", "vectors.tar.gz"])

chdir("{}/app/controllers/res/rosbag".format(rootpath))
call(["wget", "https://autoware.blob.core.windows.net/web-ui-resouces/bagfile.tar.gz"])
call(["tar", "-zxvf", "bagfile.tar.gz"])

chdir("{}/app/controllers/res/mission".format(rootpath))
call(["wget", "https://autoware.blob.core.windows.net/web-ui-resouces/waypoints.csv"])

# make_launch_files.py
chdir("{}/app/controllers".format(rootpath))
call(["python", "make_launch_files.py"])

# env
chdir("{}/app/views".format(rootpath))
call(["ln", "-s", "../config/.env", ".env"])
chdir("{}/app/config".format(rootpath))
call(["ln", "-s", "sample.env", ".env"])

# gulp browserify
chdir("{}/app/views".format(rootpath))
call(["gulp", "browserify"])

