#!/bin/sh

# Build Docker Image
if ([ "$1" = "kinetic" ] || [ "$1" = "indigo" ]) && ([ "$2" = "" ] || [ $2 = "nvidia" ])
then
    echo "Use $1"
    docker build -t autoware-$1 -f Dockerfile.$1 .
    if [ $2 = "nvidia" ]; then
      docker build --build-arg FROM_ARG=autoware-$1 -t autoware-$1-$2 -f Dockerfile.nvidia-docker-v2 .
    fi
else
    echo "Select distribution as first argument 'kinetic' or 'indigo' and optionally 'nvidia' as 2nd argument to target nvidia docker v2"
fi
