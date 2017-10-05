#!/bin/bash

DIR=$(cd $(dirname $0) ; pwd)

if [ $# -lt 4 ]; then
  echo "Usage: $0 <port> <baud> <mode> <freqency> <wait_sec>"
  echo "   ex. $0 /dev/ttyUSB0 115200 2 100 3"
  exit 1
fi

PORT=$1
BAUD=$2
MODE=$3
FREQ=$4
WSEC=$5

${DIR}/add_perm.sh 0666 ${PORT} || exit 1

ARGS="port:=${PORT} baudrate:=${BAUD} mode:=${MODE} frequency:=${FREQ}"

roslaunch xsens_driver mti300.launch ${ARGS} boot_device:=true
sleep ${WSEC}
roslaunch xsens_driver mti300.launch ${ARGS} boot_device:=false

# EOF
