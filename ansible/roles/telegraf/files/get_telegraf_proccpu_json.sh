#!/bin/bash

SAMPLING_SEC=5

echo "{"
pidstat -u -h -l "${SAMPLING_SEC}" 1 |
    tail -n +4 |
    awk '{ cpu=$8; $1=$2=$3=$4=$5=$6=$7=$8=$9=""; print cpu,$0 }' |
    sort -n |
    while read -r cpu cmd; do
        if [[ ${cpu%%.*} -le 0 ]]; then
            continue
        fi
        cmd="${cmd// /_}"
        cmd="${cmd//=/_}"
        cmd="${cmd:0:50}"
        echo "\"${cmd}\":${cpu},"
    done
echo '"z":0'
echo "}"
