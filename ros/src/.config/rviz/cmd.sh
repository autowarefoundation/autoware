#!/bin/bash

DIR=$(cd $(dirname $0) ; pwd)

REMOTE=""
KEYFILE=""
if [ -e $DIR/host ]; then
  REMOTE=$(sed -n 's/^host *: *//p' $DIR/host)
  if [ "$REMOTE" = "localhost" ]; then
    REMOTE=""
  fi
  KEYFILE=$(sed -n 's/^key *: *//p' $DIR/host)
fi

#echo REMOTE=[$REMOTE]
#echo KEYFILE=[$KEYFILE]
#echo "$DIR/default.rviz"

if [ x"$REMOTE" = x ]; then
    rosrun rviz rviz  -d "$DIR/default.rviz"
else
  KEYOPT=""
  if [ x"$KEYFILE" != x ]; then
    KEYOPT="-i $KEYFILE"
  fi
  ssh -tt $KEYOPT $REMOTE <<EOF
    [ -d /opt/ros/indigo ] && . /opt/ros/indigo/setup.bash
    [ -d /opt/ros/jade ] && . /opt/ros/jade/setup.bash
    [ -d $DIR/../../../devel ] && . $DIR/../../../devel/setup.bash || \
      echo "$REMOTE:$DIR/../../../devel: no such directory"
    ROS_IP=$REMOTE
    ROS_MASTER_URI=$ROS_MASTER_URI
    DISPLAY=:0
    export ROS_IP ROS_MASTER_URI DISPLAY
    rosrun rviz rviz -d "$DIR/default.rviz"
    #xeyes
EOF
fi

# EOF
