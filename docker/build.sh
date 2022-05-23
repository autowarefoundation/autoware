#!/usr/bin/env bash

set -e

SCRIPT_DIR=$(readlink -f "$(dirname "$0")")
WORKSPACE_ROOT="$SCRIPT_DIR/../"

# Load env
source "$WORKSPACE_ROOT/amd64.env"
if [ "$(uname -m)" = "aarch64" ]; then
    source "$WORKSPACE_ROOT/arm64.env"
fi

# https://github.com/docker/buildx/issues/484
export BUILDKIT_STEP_LOG_MAX_SIZE=10000000

docker buildx bake --load --progress=plain -f "$SCRIPT_DIR/autoware-universe/docker-bake.hcl" \
    --set "*.context=$WORKSPACE_ROOT" \
    --set "*.ssh=default" \
    --set "*.args.ROS_DISTRO=$rosdistro" \
    --set "*.args.CUDA_IMAGE_TAG=$cuda_image_tag" \
    --set "*.args.CUDNN_VERSION=$cudnn_version" \
    --set "*.args.TENSORRT_VERSION=$tensorrt_version" \
    --set "devel.tags=ghcr.io/autowarefoundation/autoware-universe:$rosdistro-latest" \
    --set "prebuilt.tags=ghcr.io/autowarefoundation/autoware-universe:$rosdistro-latest-prebuilt"
