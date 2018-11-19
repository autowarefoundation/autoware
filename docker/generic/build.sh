#!/bin/sh

# Build Docker Image
if [ "$1" = "kinetic" ]
then
    echo "Use $1"
    nvidia-docker build -t autoware-$1 -f Dockerfile.$1 ./../.. --no-cache
elif [ "$1" = "indigo" ]
then
    echo "Indigo is deprecated and will be removed in a future release, please use Kinetic instead"
    nvidia-docker build -t autoware-$1 -f Dockerfile.$1 ./../.. --no-cache
else
    echo "Select distribution, kinetic|indigo"
fi
