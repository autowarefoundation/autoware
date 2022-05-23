#!/usr/bin/env bash

set -e

SCRIPT_DIR=$(readlink -f "$(dirname "$0")")
WORKSPACE_ROOT="$SCRIPT_DIR/../"

# Set default platform
platform="linux/amd64"
if [ "$(uname -m)" = "aarch64" ]; then
    platform="linux/arm64"
fi

# Override platform by arg
if [ "$1" = "amd64" ] || [ "$1" = "arm64" ]; then
    platform="linux/$1"
fi

# Load env
source "$WORKSPACE_ROOT/amd64.env"
if [ "$platform" = "linux/arm64" ]; then
    source "$WORKSPACE_ROOT/arm64.env"
fi

# https://github.com/docker/buildx/issues/484
export BUILDKIT_STEP_LOG_MAX_SIZE=10000000

docker buildx bake --load --progress=plain -f "$SCRIPT_DIR/autoware-universe/docker-bake.hcl" \
    --set "*.context=$WORKSPACE_ROOT" \
    --set "*.ssh=default" \
    --set "*.platform=$platform" \
    --set "*.args.ROS_DISTRO=$rosdistro" \
    --set "*.args.BASE_IMAGE=$base_image" \
    --set "devel.tags=ghcr.io/autowarefoundation/autoware-universe:$rosdistro-latest" \
    --set "prebuilt.tags=ghcr.io/autowarefoundation/autoware-universe:$rosdistro-latest-prebuilt"
