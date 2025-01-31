#! /bin/bash

docker run -it --rm \
    --net=host \
    --platform linux/amd64 \
    --name web-visualizer \
    --env VNC_ENABLED=true \
    --env VNC_PASSWORD=openadkit \
    ghcr.io/autowarefoundation/autoware:web-visualizer-20250130-amd64
