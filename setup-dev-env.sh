#!/usr/bin/env bash
# Set up development environment for Autoware Core/Universe.
# Usage: setup-dev-env.sh <ros2_installation_type('core' or 'universe')> [-y] [-v] [--no-nvidia]
# Note: -y option is only for CI.

set -e

# Function to print help message
print_help() {
    echo "Usage: setup-dev-env.sh [OPTIONS] [COMMAND]"
    echo "Commands:"
    echo "  check           Check the development environment configuration"
    echo "Options:"
    echo "  --help          Display this help message"
    echo "  -h              Display this help message"
    echo "  -y              Use non-interactive mode"
    echo "  -v              Enable debug outputs"
    echo "  --no-nvidia     Disable installation of the NVIDIA-related roles ('cuda' and 'tensorrt')"
    echo "  --no-cuda-drivers Disable installation of 'cuda-drivers' in the role 'cuda'"
    echo "  --runtime       Disable installation dev package of role 'cuda' and 'tensorrt'"
    echo "  --data-dir      Set data directory (default: $HOME/autoware_data)"
    echo "  --download-artifacts"
    echo "                  Download artifacts"
    echo "  --module        Specify the module (default: all)"
    echo "  --ros-distro    Specify ROS distribution (humble or jazzy, default: humble)"
    echo ""
}

SCRIPT_DIR=$(readlink -f "$(dirname "$0")")

# Function to check environment
check_environment() {
    echo "Checking development environment..."
    local errors=0

    # Check OS
    if [ -f /etc/os-release ]; then
        source /etc/os-release
        if [[ "$VERSION_ID" != "22.04" && "$VERSION_ID" != "24.04" ]]; then
            echo -e "\e[31m[ERROR] Unsupported OS version: $VERSION_ID. Only 22.04 and 24.04 are supported.\e[0m"
            errors=$((errors + 1))
        else
            echo -e "\e[32m[OK] OS version: $VERSION_ID\e[0m"
        fi
    else
        echo -e "\e[31m[ERROR] Cannot determine OS version.\e[0m"
        errors=$((errors + 1))
    fi

    # Check critical tools
    for tool in git sudo python3; do
        if command -v $tool >/dev/null 2>&1; then
            echo -e "\e[32m[OK] $tool is installed.\e[0m"
        else
            echo -e "\e[31m[ERROR] $tool is missing.\e[0m"
            errors=$((errors + 1))
        fi
    done

    # Check pipx
    if command -v pipx >/dev/null 2>&1; then
        echo -e "\e[32m[OK] pipx is installed.\e[0m"
    else
        echo -e "\e[33m[WARNING] pipx is not in PATH. It might be installed but not in PATH.\e[0m"
        # Try to find it in likely locations
        if [ -f "$HOME/.local/bin/pipx" ]; then
             echo -e "\e[33m[INFO] pipx found at $HOME/.local/bin/pipx.\e[0m"
        else
             errors=$((errors + 1))
        fi
    fi

    # Check ansible
    if command -v ansible-playbook >/dev/null 2>&1; then
        echo -e "\e[32m[OK] ansible-playbook is installed.\e[0m"
    else
        echo -e "\e[31m[ERROR] ansible-playbook is missing or not in PATH.\e[0m"
        errors=$((errors + 1))
    fi
    
    if [ $errors -eq 0 ]; then
        echo -e "\e[32mEnvironment check passed.\e[0m"
        return 0
    else
        echo -e "\e[31mEnvironment check failed with $errors errors.\e[0m"
        return 1
    fi
}

# Parse arguments
args=()
option_data_dir="$HOME/autoware_data"
mode="install"

while [ "$1" != "" ]; do
    case "$1" in
    check)
        mode="check"
        ;;
    --help | -h)
        print_help
        exit 1
        ;;
    -y)
        # Use non-interactive mode.
        option_yes=true
        ;;
    -v)
        # Enable debug outputs.
        option_verbose=true
        ;;
    --no-nvidia)
        # Disable installation of the NVIDIA-related roles ('cuda' and 'tensorrt').
        option_no_nvidia=true
        ;;
    --no-cuda-drivers)
        # Disable installation of 'cuda-drivers' in the role 'cuda'.
        option_no_cuda_drivers=true
        ;;
    --runtime)
        # Disable installation dev package of role 'cuda' and 'tensorrt'.
        option_runtime=true
        ;;
    --data-dir)
        # Set data directory
        option_data_dir="$2"
        shift
        ;;
    --download-artifacts)
        # Set download artifacts option
        option_download_artifacts=true
        ;;
    --module)
        option_module="$2"
        shift
        ;;
    --ros-distro)
        option_ros_distro="$2"
        shift
        ;;
    *)
        args+=("$1")
        ;;
    esac
    shift
done

if [ "$mode" == "check" ]; then
    check_environment
    exit $?
fi

# Select installation type
target_playbook="autoware.dev_env.universe" # default

if [ ${#args[@]} -ge 1 ]; then
    target_playbook="autoware.dev_env.${args[0]}"
fi

# Initialize ansible args
ansible_args=()

# Confirm to start installation
if [ "$option_yes" = "true" ]; then
    echo -e "\e[36mRun the setup in non-interactive mode.\e[m"
else
    echo -e "\e[33mSetting up the build environment can take up to 1 hour.\e[m"
    read -rp ">  Are you sure you want to run setup? [y/N] " answer

    # Check whether to cancel
    if ! [[ ${answer:0:1} =~ y|Y ]]; then
        echo -e "\e[33mCancelled.\e[0m"
        exit 1
    fi

    ansible_args+=("--ask-become-pass")
fi

# Check verbose option
if [ "$option_verbose" = "true" ]; then
    ansible_args+=("-vvv")
fi

# Check installation of NVIDIA libraries
if [ "$option_no_nvidia" = "true" ]; then
    ansible_args+=("--extra-vars" "prompt_install_nvidia=n")
elif [ "$option_yes" = "true" ]; then
    ansible_args+=("--extra-vars" "prompt_install_nvidia=y")
fi

# Check installation of CUDA Drivers
if [ "$option_no_cuda_drivers" = "true" ]; then
    ansible_args+=("--extra-vars" "cuda_install_drivers=false")
fi

# Check installation of dev package
if [ "$option_runtime" = "true" ]; then
    ansible_args+=("--extra-vars" "ros2_installation_type=ros-base") # ROS installation type, default "desktop"
    ansible_args+=("--extra-vars" "install_devel=N")
else
    ansible_args+=("--extra-vars" "install_devel=y")
fi

# Check downloading artifacts
if [ "$target_playbook" = "autoware.dev_env.openadkit" ]; then
    if [ "$option_download_artifacts" = "true" ]; then
        echo -e "\e[36mArtifacts will be downloaded to $option_data_dir\e[m"
        ansible_args+=("--extra-vars" "prompt_download_artifacts=y")
    else
        ansible_args+=("--extra-vars" "prompt_download_artifacts=N")
    fi
elif [ "$option_yes" = "true" ] || [ "$option_download_artifacts" = "true" ]; then
    echo -e "\e[36mArtifacts will be downloaded to $option_data_dir\e[m"
    ansible_args+=("--extra-vars" "prompt_download_artifacts=y")
fi

ansible_args+=("--extra-vars" "data_dir=$option_data_dir")

# Check module option
if [ "$option_module" != "" ]; then
    ansible_args+=("--extra-vars" "module=$option_module")
fi

# Check ros-distro option
if [ "$option_ros_distro" != "" ]; then
    export ROS_DISTRO="$option_ros_distro"
    ansible_args+=("--extra-vars" "rosdistro=$option_ros_distro")
fi

# Load env
source "$SCRIPT_DIR/amd64.env"
env_file="$SCRIPT_DIR/amd64.env"
if [ "$option_ros_distro" = "jazzy" ]; then
    source "$SCRIPT_DIR/amd64_jazzy.env"
    env_file="$SCRIPT_DIR/amd64_jazzy.env"
fi

if [ "$(uname -m)" = "aarch64" ]; then
    source "$SCRIPT_DIR/arm64.env"
fi

# Add env args
# shellcheck disable=SC2013
for env_name in $(sed -e "s/^\s*//" -e "/^#/d" -e "s/=.*//" <"$env_file"); do
    ansible_args+=("--extra-vars" "${env_name}=${!env_name}")
done

# Install sudo
if ! (command -v sudo >/dev/null 2>&1); then
    apt-get -y update
    apt-get -y install sudo
fi

# Install git
if ! (command -v git >/dev/null 2>&1); then
    sudo apt-get -y update
    sudo apt-get -y install git
fi

# Install pip for ansible
if ! (python3 -m pip --version >/dev/null 2>&1); then
    sudo apt-get -y update
    sudo apt-get -y install python3-pip python3-venv
fi

# Install pipx for ansible
if ! (python3 -m pipx --version >/dev/null 2>&1); then
    sudo apt-get -y update
    sudo apt-get -y install pipx
fi

# Install ansible
python3 -m pipx ensurepath
export PATH="${PIPX_BIN_DIR:=$HOME/.local/bin}:$PATH"
pipx install --include-deps --force "ansible==10.*"

# Install ansible collections
echo -e "\e[36m"ansible-galaxy collection install -f -r "$SCRIPT_DIR/ansible-galaxy-requirements.yaml" "\e[m"
ansible-galaxy collection install -f -r "$SCRIPT_DIR/ansible-galaxy-requirements.yaml"

# Run ansible
echo -e "\e[36m"ansible-playbook "$target_playbook" "${ansible_args[@]}" "\e[m"
if ansible-playbook "$target_playbook" "${ansible_args[@]}"; then
    echo -e "\e[32mCompleted.\e[0m"
    exit 0
else
    echo -e "\e[31mFailed.\e[0m"
    exit 1
fi
