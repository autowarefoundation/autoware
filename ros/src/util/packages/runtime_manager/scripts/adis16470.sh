#!/bin/bash

DIR=$(cd $(dirname $0) ; pwd)

if [ $# -lt 2 ]; then
  echo "Usage: $0 <port> <freqency>"
  echo "   ex. $0 /dev/ttyACM0 100"
  exit 1
fi

PORT=$1
FREQ=$2

${DIR}/add_perm.sh 0666 ${PORT} || exit 1

roslaunch adi_driver adis16470.launch device:=${PORT} rate:=${FREQ}

# EOF
