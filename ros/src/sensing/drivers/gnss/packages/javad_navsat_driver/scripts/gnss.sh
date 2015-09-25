#!/bin/bash

cwd=`dirname "${0}"`
port=`echo $1 | cut -d "=" -f2`
baud=`echo $2 | cut -d "=" -f2`
echo "port : $port , baud : $baud"

#echo ""enable command" > $port"
cat $cwd/enable.txt > $port

sleep 1

echo "launch gnss driver..."
roslaunch javad_navsat_driver javad_navsat.launch port:=$port baud:=$baud
