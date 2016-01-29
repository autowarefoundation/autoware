#!/bin/bash

if [ $# -lt 1 ]; then
  echo "Usage $0 <start|stop>"
  exit 0
fi

DIR=$(cd $(dirname $0) ; pwd)

REMOTE=""
KEYFILE=""
if [ -e $DIR/host ]; then
  REMOTE=$(sed -n 's/^host *: *//p' $DIR/host)
  if [ $REMOTE = localhost ]; then
    REMOTE=""
  fi
  KEYFILE=$(sed -n 's/^key *: *//p' $DIR/host)
fi

#echo REMOTE=[$REMOTE]
#echo KEYFILE=[$KEYFILE]

if [ x"$REMOTE" = x ]; then
  if [ $1 = start ]; then
    rosrun rviz rviz
  fi
else
  KEYOPT=""
  if [ x"$KEYFILE" != x ]; then
    KEYOPT="-i $KEYFILE"
  fi
  if [ $1 = start ]; then
    cat <<EOF | ssh $KEYOPT $REMOTE
    [ -d /opt/ros/indigo ] && . /opt/ros/indigo/setup.bash
    [ -d /opt/ros/jade ] && . /opt/ros/jade/setup.bash
    ROS_IP=$REMOTE
    ROS_MASTER_URI=$ROS_MASTER_URI
    DISPLAY=:0
    export ROS_IP ROS_MASTER_URI DISPLAY
    rosrun rviz rviz
    #xeyes
EOF
  else
    ssh $KEYOPT $REMOTE pkill rviz
    #ssh $KEYOPT $REMOTE pkill xeyes
  fi
fi

# EOF
