#!/usr/bin/env python
# coding: utf-8

from subprocess import call
from os import getcwd, chdir

rootpath = getcwd()
print rootpath

# pip install
call(["sudo", "pip", "install", "-r", "requirements.txt"])

# npm install
chdir("{}/app/views".format(rootpath))
call(["npm", "install"])

# make_launch_files.py
chdir("{}/app/controllers".format(rootpath))
call(["python", "make_launch_files.py"])

# env
chdir("{}/app".format(rootpath))
call(["ln", "-s", "./config/.env", "./views/.env"])

chdir("{}/app/config".format(rootpath))
call(["ln", "-s", "sample.env", ".env"])

# gulp browserify
chdir("{}/app/views".format(rootpath))
call(["gulp", "browserify"])

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

