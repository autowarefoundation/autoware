#!/bin/bash

if [ $# -lt 2 ]; then
  echo "Usage: $0 <add_perm> <file>"
  echo "   ex. $0 0666 /dev/ttyUSB0"
  exit 1
fi

P=$1
F=$2

if [ ! -e $F ]; then
  echo "Not found $F"
  exit 1
fi

S="0$(stat -c %a $F)" # ex. "0666"

if [ $(($S & $P)) -ne $(($P)) ]; then
  M=$(echo "obase=8; $(($S | $P))" | bc)
  echo "Please input password for chmod $F"
  sudo chmod "0$M" $F || exit 1


fi

exit 0

# EOF
