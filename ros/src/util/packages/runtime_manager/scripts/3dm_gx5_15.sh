#!/bin/bash

DIR=$(cd $(dirname $0) ; pwd)

if [ $# -lt 3 ]; then
  echo "Usage: $0 <port> <baud> <freqency>"
  echo "   ex. $0 /dev/ttyACM0 115200 100"
  exit 1
fi

PORT=$1
BAUD=$2
FREQ=$3

${DIR}/add_perm.sh 0666 ${PORT} || exit 1

roslaunch microstrain_driver 3dm_gx5_15.launch port:=${PORT} baudrate:=${BAUD} frequency:=${FREQ}

# EOF
