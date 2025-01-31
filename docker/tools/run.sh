#! /bin/bash

# docker run -it --rm \
#     --net=host \
#     --platform linux/amd64 \
#     --name visualizer \
#     --env WEB_ENABLED=true \
#     --env WEB_PASSWORD=openadkit \
#     ghcr.io/autowarefoundation/autoware:visualizer-20250130-amd64


docker run -it --rm \
    --net=host \
    --platform linux/amd64 \
    --name simulator \
    ghcr.io/autowarefoundation/autoware:simulator-20250130-amd64
