#!/usr/bin/env bash

set -e

# Function to print help message
print_help() {
    echo "Usage: build-jazzy.sh [OPTIONS]"
    echo "Options:"
    echo "  --help          Display this help message"
    echo "  -h              Display this help message"
    echo "  --no-cuda       Disable CUDA support"
    echo "  --platform      Specify the platform (default: current platform)"
    echo "  --devel-only    Build devel image only"
    echo "  --target        Specify the target image (default: universe or universe-devel if --devel-only is set)"
    echo ""
    echo "Note: The --platform option should be 'linux/amd64'."
}

SCRIPT_DIR=$(readlink -f "$(dirname "$0")")
WORKSPACE_ROOT="$SCRIPT_DIR/.."

# Parse arguments
parse_arguments() {
    while [ "$1" != "" ]; do
        case "$1" in
        --help | -h)
            print_help
            exit 1
            ;;
        --no-cuda)
            option_no_cuda=true
            ;;
        --platform)
            option_platform="$2"
            shift
            ;;
        --devel-only)
            option_devel_only=true
            ;;
        --target)
            option_target="$2"
            shift
            ;;
        *)
            echo "Unknown option: $1"
            print_help
            exit 1
            ;;
        esac
        shift
    done
}

# Set CUDA options
set_cuda_options() {
    if [ "$option_no_cuda" = "true" ]; then
        setup_args="--no-nvidia"
        image_name_suffix=""
    else
        image_name_suffix="-cuda"
    fi
}

# Set build options
set_build_options() {
    if [ -n "$option_target" ]; then
        target="$option_target"
        image_name_suffix=""
    else
        if [ "$option_devel_only" = "true" ]; then
            target="universe-devel"
        else
            target="universe"
        fi
    fi
}

# Set platform
set_platform() {
    if [ -n "$option_platform" ]; then
        platform="$option_platform"
    else
        platform="linux/amd64"
    fi
}

# Set arch lib dir
set_arch_lib_dir() {
    if [ "$platform" = "linux/amd64" ]; then
        lib_dir="x86_64"
    else
        echo "Unsupported platform: $platform"
        exit 1
    fi
}

# Load env
load_env() {
    source "$WORKSPACE_ROOT/amd64-jazzy.env"
}

# Clone repositories
clone_repositories() {
    cd "$WORKSPACE_ROOT"
    if [ ! -d "src" ]; then
        mkdir -p src
        vcs import src <autoware.repos
        vcs import src <extra-packages.repos
    else
        echo "Source directory already exists. Updating repositories..."
        vcs import src <autoware.repos
        vcs import src <extra-packages.repos
        vcs pull src
    fi
}

# Build images
build_images() {
    # https://github.com/docker/buildx/issues/484
    export BUILDKIT_STEP_LOG_MAX_SIZE=10000000

    echo "Building jazzy images for platform: $platform"
    echo "ROS distro: $rosdistro"
    echo "Base image: $base_image"
    echo "Setup args: $setup_args"
    echo "Lib dir: $lib_dir"
    echo "Image name suffix: $image_name_suffix"
    echo "Target: $target"

    set -x
    # Build base images
    if [ "$option_no_cuda" = "true" ]; then
        docker buildx bake --load --progress=plain -f "$SCRIPT_DIR/docker-bake-base-jazzy.hcl" \
            --set "*.context=$WORKSPACE_ROOT" \
            --set "*.ssh=default" \
            --set "*.platform=$platform" \
            --set "*.args.ROS_DISTRO=$rosdistro" \
            --set "*.args.BASE_IMAGE=$base_image" \
            --set "*.args.SETUP_ARGS=$setup_args" \
            --set "*.args.LIB_DIR=$lib_dir" \
            --set "base-jazzy.tags=ghcr.io/autowarefoundation/autoware-base:jazzy-latest" \
            base-jazzy
    else
        docker buildx bake --load --progress=plain -f "$SCRIPT_DIR/docker-bake-base-jazzy.hcl" \
            --set "*.context=$WORKSPACE_ROOT" \
            --set "*.ssh=default" \
            --set "*.platform=$platform" \
            --set "*.args.ROS_DISTRO=$rosdistro" \
            --set "*.args.BASE_IMAGE=$base_image" \
            --set "*.args.SETUP_ARGS=$setup_args" \
            --set "*.args.LIB_DIR=$lib_dir" \
            --set "base-jazzy.tags=ghcr.io/autowarefoundation/autoware-base:jazzy-latest" \
            --set "base-jazzy-cuda.tags=ghcr.io/autowarefoundation/autoware-base:jazzy-cuda-latest"
    fi
    
    # Build target images
    if [ "$option_no_cuda" = "true" ]; then
        docker buildx bake --load --progress=plain -f "$SCRIPT_DIR/docker-bake-jazzy.hcl" \
            --set "*.context=$WORKSPACE_ROOT" \
            --set "*.ssh=default" \
            --set "*.platform=$platform" \
            --set "*.args.ROS_DISTRO=$rosdistro" \
            --set "*.args.AUTOWARE_BASE_IMAGE=$autoware_base_image" \
            --set "*.args.AUTOWARE_BASE_CUDA_IMAGE=$autoware_base_cuda_image" \
            --set "*.args.SETUP_ARGS=$setup_args" \
            --set "*.args.LIB_DIR=$lib_dir" \
            --set "universe-sensing-perception-devel.tags=ghcr.io/autowarefoundation/autoware:jazzy-universe-sensing-perception-devel" \
            --set "universe-sensing-perception.tags=ghcr.io/autowarefoundation/autoware:jazzy-universe-sensing-perception" \
            --set "universe-localization-mapping-devel.tags=ghcr.io/autowarefoundation/autoware:jazzy-universe-localization-mapping-devel" \
            --set "universe-localization-mapping.tags=ghcr.io/autowarefoundation/autoware:jazzy-universe-localization-mapping" \
            --set "universe-planning-control-devel.tags=ghcr.io/autowarefoundation/autoware:jazzy-universe-planning-control-devel" \
            --set "universe-planning-control.tags=ghcr.io/autowarefoundation/autoware:jazzy-universe-planning-control" \
            --set "universe-vehicle-system-devel.tags=ghcr.io/autowarefoundation/autoware:jazzy-universe-vehicle-system-devel" \
            --set "universe-vehicle-system.tags=ghcr.io/autowarefoundation/autoware:jazzy-universe-vehicle-system" \
            --set "universe-visualization-devel.tags=ghcr.io/autowarefoundation/autoware:jazzy-universe-visualization-devel" \
            --set "universe-visualization.tags=ghcr.io/autowarefoundation/autoware:jazzy-universe-visualization" \
            --set "universe-devel.tags=ghcr.io/autowarefoundation/autoware:jazzy-universe-devel" \
            --set "universe.tags=ghcr.io/autowarefoundation/autoware:jazzy-universe" \
            --set "core-common-devel.tags=ghcr.io/autowarefoundation/autoware:jazzy-core-common-devel" \
            "$target"
    else
        docker buildx bake --load --progress=plain -f "$SCRIPT_DIR/docker-bake-jazzy.hcl" -f "$SCRIPT_DIR/docker-bake-jazzy-cuda.hcl" \
            --set "*.context=$WORKSPACE_ROOT" \
            --set "*.ssh=default" \
            --set "*.platform=$platform" \
            --set "*.args.ROS_DISTRO=$rosdistro" \
            --set "*.args.AUTOWARE_BASE_IMAGE=$autoware_base_image" \
            --set "*.args.AUTOWARE_BASE_CUDA_IMAGE=$autoware_base_cuda_image" \
            --set "*.args.SETUP_ARGS=$setup_args" \
            --set "*.args.LIB_DIR=$lib_dir" \
            --set "universe-sensing-perception-devel.tags=ghcr.io/autowarefoundation/autoware:jazzy-universe-sensing-perception-devel" \
            --set "universe-sensing-perception.tags=ghcr.io/autowarefoundation/autoware:jazzy-universe-sensing-perception" \
            --set "universe-localization-mapping-devel.tags=ghcr.io/autowarefoundation/autoware:jazzy-universe-localization-mapping-devel" \
            --set "universe-localization-mapping.tags=ghcr.io/autowarefoundation/autoware:jazzy-universe-localization-mapping" \
            --set "universe-planning-control-devel.tags=ghcr.io/autowarefoundation/autoware:jazzy-universe-planning-control-devel" \
            --set "universe-planning-control.tags=ghcr.io/autowarefoundation/autoware:jazzy-universe-planning-control" \
            --set "universe-vehicle-system-devel.tags=ghcr.io/autowarefoundation/autoware:jazzy-universe-vehicle-system-devel" \
            --set "universe-vehicle-system.tags=ghcr.io/autowarefoundation/autoware:jazzy-universe-vehicle-system" \
            --set "universe-visualization-devel.tags=ghcr.io/autowarefoundation/autoware:jazzy-universe-visualization-devel" \
            --set "universe-visualization.tags=ghcr.io/autowarefoundation/autoware:jazzy-universe-visualization" \
            --set "universe-devel.tags=ghcr.io/autowarefoundation/autoware:jazzy-universe-devel" \
            --set "universe.tags=ghcr.io/autowarefoundation/autoware:jazzy-universe" \
            --set "universe-sensing-perception-devel-cuda.tags=ghcr.io/autowarefoundation/autoware:jazzy-universe-sensing-perception-devel-cuda" \
            --set "universe-sensing-perception-cuda.tags=ghcr.io/autowarefoundation/autoware:jazzy-universe-sensing-perception-cuda" \
            --set "universe-devel-cuda.tags=ghcr.io/autowarefoundation/autoware:jazzy-universe-devel-cuda" \
            --set "universe-cuda.tags=ghcr.io/autowarefoundation/autoware:jazzy-universe-cuda" \
            "$target$image_name_suffix"
    fi
    set +x
}

# Remove dangling images
remove_dangling_images() {
    docker image prune -f
}

# Main script execution
parse_arguments "$@"
set_cuda_options
set_build_options
set_platform
set_arch_lib_dir
load_env
clone_repositories
build_images
remove_dangling_images
