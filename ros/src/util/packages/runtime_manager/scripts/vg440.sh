#!/bin/bash

DIR=$(cd $(dirname $0) ; pwd)

if [ $# -lt 2 ]; then
  echo "Usage: $0 <port> <baud>"
  echo "   ex. $0 /dev/ttyUSB0 57600"
  exit 1
fi

PORT=$1
BAUD=$2

${DIR}/add_perm.sh 0666 ${PORT} || exit 1

roslaunch memsic_imu vg440.launch port:=${PORT} baudrate:=${BAUD}

# EOF
