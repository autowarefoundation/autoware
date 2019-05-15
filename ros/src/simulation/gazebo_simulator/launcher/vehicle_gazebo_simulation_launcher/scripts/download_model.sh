#!/bin/sh

mkdir -p ~/.gazebo/models
echo "---- Download generic gazebo models ----"
if [ -e /tmp/gazebo_models ]; then
  echo "---- Already exist /tmp/gazebo_models ----"
  echo "---- removing /tmp/gazebo_models ----"
  rm -r /tmp/gazebo_models
fi
hg clone https://bitbucket.org/osrf/gazebo_models /tmp/gazebo_models

if [ -e /tmp/gazebo_models ]; then
  echo "---- Install generic gazebo models ----"
  mv -n /tmp/gazebo_models/* ~/.gazebo/models/
  echo "---- Completed ----"
else
  echo "---- Error ----"
fi

echo "---- Install models ----"
cp -rn $(rospack find gazebo_world_description)/models/* ~/.gazebo/models/
echo "---- Completed ----"
