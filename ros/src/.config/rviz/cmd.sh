#!/bin/bash

DIR=$(cd $(dirname $0) ; pwd)

REMOTE=""
KEYFILE=""
if [ -e $DIR/host ]; then
  REMOTE=$((sed -n 's/^host *: *//p' $DIR/host ; \
            grep -v -e '^#' -e ':' -e '^$' $DIR/host) | tail -1)
  if [ "$REMOTE" = "localhost" ]; then
    REMOTE=""
  fi
  KEYFILE=$(sed -n 's/^key *: *//p' $DIR/host)
fi

#echo REMOTE=[$REMOTE]
#echo KEYFILE=[$KEYFILE]

if [ x"$REMOTE" = x ]; then
    rosrun rviz rviz
else
  KEYOPT=""
  if [ x"$KEYFILE" != x ]; then
    KEYOPT="-i $KEYFILE"
  fi
  [ x"$REMOTE_DISPLAY" = x ] && REMOTE_DISPLAY=:0
  XOPT=""
  [ "$REMOTE_DISPLAY" = "-" ] && XOPT="-X"

  setsid ssh -tt $XOPT $KEYOPT $REMOTE <<EOF
    [ -d /opt/ros/indigo ] && . /opt/ros/indigo/setup.bash
    [ -d /opt/ros/jade ] && . /opt/ros/jade/setup.bash
    [ -d $DIR/../../../devel ] && . $DIR/../../../devel/setup.bash || \
      echo "$REMOTE:$DIR/../../../devel: no such directory"
    ROS_IP=\$(hostname -I)
    FROM_IP=\$(echo \$SSH_CONNECTION | cut -d ' ' -f 1)
    ROS_MASTER_URI=http://\$FROM_IP:11311
    [ "$REMOTE_DISPLAY" != "-" ] && DISPLAY=$REMOTE_DISPLAY
    export ROS_IP ROS_MASTER_URI DISPLAY
    export GNOME_DESKTOP_SESSION_ID="this-is-deprecated"
    rosrun rviz rviz
    #xeyes
EOF
fi

# EOF
