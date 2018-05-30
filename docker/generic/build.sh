#!/bin/sh

# Build Docker Image
if [ "$1" = "kinetic" ] || [ "$1" = "indigo" ]
then
    echo "Use $1"
    nvidia-docker build -t autoware-$1 -f Dockerfile.$1 . --no-cache
else
    echo "Select distribution, kinetic|indigo"
fi
