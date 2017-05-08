#!/bin/bash

DIR=$(cd $(dirname $0) ; pwd)

if [ $# -lt 4 ]; then
  echo "Usage: $0 <port> <baud> <mode> <freqency>"
  echo "   ex. $0 /dev/ttyUSB0 115200 2 100"
  exit 1
fi

PORT=$1
BAUD=$2
MODE=$3
FREQ=$4

${DIR}/add_perm.sh 0666 ${PORT} || exit 1

roslaunch runtime_manager mti300.launch port:=${PORT} baudrate:=${BAUD} mode:=${MODE} frequency:=${FREQ}

# EOF
