#!/usr/bin/env bash

set -e

# Function to print help message
print_help() {
    echo "Usage: build.sh [OPTIONS]"
    echo "Options:"
    echo "  --help          Display this help message"
    echo "  -h              Display this help message"
    echo "  --no-cuda       Disable CUDA support"
    echo "  --platform      Specify the platform (default: current platform)"
    echo "  --devel-only    Build devel image only"
    echo "  --target        Specify the target image (default: universe or universe-devel if --devel-only is set)"
    echo "  --ros-distro    Specify ROS distribution (humble or jazzy, default: humble)"
    echo ""
    echo "Note: The --platform option should be one of 'linux/amd64' or 'linux/arm64'."
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
        --ros-distro)
            option_ros_distro="$2"
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

# Set ROS distribution
set_ros_distro() {
    if [ -n "$option_ros_distro" ]; then
        ros_distro="$option_ros_distro"
    else
        ros_distro="humble"
    fi
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

# Note: Image tags are loaded from env files (amd64.env or amd64_jazzy.env)
# via the load_env() function, which sets $autoware_base_image and $autoware_base_cuda_image

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
        if [ "$(uname -m)" = "aarch64" ]; then
            platform="linux/arm64"
        fi
    fi
}

# Set arch lib dir
set_arch_lib_dir() {
    if [ "$platform" = "linux/arm64" ]; then
        lib_dir="aarch64"
    elif [ "$platform" = "linux/amd64" ]; then
        lib_dir="x86_64"
    else
        echo "Unsupported platform: $platform"
        exit 1
    fi
}

# Load env
load_env() {
    if [ "$ros_distro" = "humble" ]; then
        source "$WORKSPACE_ROOT/amd64.env"
    else
        source "$WORKSPACE_ROOT/amd64_jazzy.env"
    fi
    if [ "$platform" = "linux/arm64" ]; then
        source "$WORKSPACE_ROOT/arm64.env"
    fi
}

# Clone repositories
clone_repositories() {
    cd "$WORKSPACE_ROOT"
    if [ ! -d "src" ]; then
        mkdir -p src
        vcs import --recursive src <repositories/autoware.repos
    else
        echo "Source directory already exists. Updating repositories..."
        vcs import --recursive src <repositories/autoware.repos
        vcs pull src
    fi
}

# Build images
build_images() {
    # https://github.com/docker/buildx/issues/484
    export BUILDKIT_STEP_LOG_MAX_SIZE=10000000

    echo "Building images for platform: $platform"
    echo "ROS distro: $rosdistro"
    echo "Base image: $base_image"
    echo "Setup args: $setup_args"
    echo "Lib dir: $lib_dir"
    echo "Image name suffix: $image_name_suffix"
    echo "Target: $target"

    set -x
    docker buildx bake --allow=ssh --load --progress=plain -f "$SCRIPT_DIR/docker-bake-base.hcl" \
        --set "*.context=$WORKSPACE_ROOT" \
        --set "*.ssh=default" \
        --set "*.platform=$platform" \
        --set "*.args.ROS_DISTRO=$rosdistro" \
        --set "*.args.BASE_IMAGE=$base_image" \
        --set "*.args.SETUP_ARGS=$setup_args" \
        --set "*.args.LIB_DIR=$lib_dir" \
        --set "base.tags=$autoware_base_image" \
        --set "base-cuda.tags=$autoware_base_cuda_image"
    docker buildx bake --allow=ssh --load --progress=plain -f "$SCRIPT_DIR/docker-bake.hcl" -f "$SCRIPT_DIR/docker-bake-cuda.hcl" \
        --set "*.context=$WORKSPACE_ROOT" \
        --set "*.ssh=default" \
        --set "*.platform=$platform" \
        --set "*.args.ROS_DISTRO=$rosdistro" \
        --set "*.args.AUTOWARE_BASE_IMAGE=$autoware_base_image" \
        --set "*.args.AUTOWARE_BASE_CUDA_IMAGE=$autoware_base_cuda_image" \
        --set "*.args.SETUP_ARGS=$setup_args" \
        --set "*.args.LIB_DIR=$lib_dir" \
        --set "universe-sensing-perception-devel.tags=ghcr.io/autowarefoundation/autoware:universe-sensing-perception-devel" \
        --set "universe-sensing-perception.tags=ghcr.io/autowarefoundation/autoware:universe-sensing-perception" \
        --set "universe-localization-mapping-devel.tags=ghcr.io/autowarefoundation/autoware:universe-localization-mapping-devel" \
        --set "universe-localization-mapping.tags=ghcr.io/autowarefoundation/autoware:universe-localization-mapping" \
        --set "universe-planning-control-devel.tags=ghcr.io/autowarefoundation/autoware:universe-planning-control-devel" \
        --set "universe-planning-control.tags=ghcr.io/autowarefoundation/autoware:universe-planning-control" \
        --set "universe-vehicle-system-devel.tags=ghcr.io/autowarefoundation/autoware:universe-vehicle-system-devel" \
        --set "universe-vehicle-system.tags=ghcr.io/autowarefoundation/autoware:universe-vehicle-system" \
        --set "universe-visualization-devel.tags=ghcr.io/autowarefoundation/autoware:universe-visualization-devel" \
        --set "universe-visualization.tags=ghcr.io/autowarefoundation/autoware:universe-visualization" \
        --set "universe-devel.tags=ghcr.io/autowarefoundation/autoware:universe-devel" \
        --set "universe.tags=ghcr.io/autowarefoundation/autoware:universe" \
        --set "universe-sensing-perception-devel-cuda.tags=ghcr.io/autowarefoundation/autoware:universe-sensing-perception-devel-cuda" \
        --set "universe-sensing-perception-cuda.tags=ghcr.io/autowarefoundation/autoware:universe-sensing-perception-cuda" \
        --set "universe-devel-cuda.tags=ghcr.io/autowarefoundation/autoware:universe-devel-cuda" \
        --set "universe-cuda.tags=ghcr.io/autowarefoundation/autoware:universe-cuda" \
        "$target$image_name_suffix"
    set +x
}

# Remove dangling images
remove_dangling_images() {
    docker image prune -f
}

# Main script execution
parse_arguments "$@"
set_ros_distro
set_cuda_options
set_build_options
set_platform
set_arch_lib_dir
load_env
clone_repositories
build_images
remove_dangling_images
