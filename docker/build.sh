#!/bin/sh

# Build Docker Image
if [ "$1" = "" ] || [ "$1" = "kinetic" ]
then
    echo "Use Kinetic"
    nvidia-docker build -t autoware-image -f Dockerfile.kinetic .
elif [ "$1" = "indigo" ]
then
    echo "Use Indigo"
    nvidia-docker build -t autoware-image -f Dockerfile.indigo .
else
    echo "Invalid argument."
fi
