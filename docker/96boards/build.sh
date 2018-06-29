#!/bin/sh

# Build Docker Image
if [ "$1" = "kinetic" ]
then
    echo "Use $1"
    docker build -t autoware-$1 -f Dockerfile.$1 . --no-cache
else
    echo "Set distribution, kinetic"
fi
