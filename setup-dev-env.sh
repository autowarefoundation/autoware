#!/usr/bin/env bash
# Set up development environment for Autoware Core/Universe.
# Usage: setup-dev-env.sh <installation_type('core' or 'universe')> [-y] [-v] [--no-nvidia]
# Note: -y option is only for CI.

set -e

SCRIPT_DIR=$(readlink -f "$(dirname "$0")")

# Parse arguments
args=()
while [ "$1" != "" ]; do
    case "$1" in
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
    --no-dev)
        # Disable installation dev packages .
        option_no_dev=true
        ;;
    *)
        args+=("$1")
        ;;
    esac
    shift
done

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
    ansible_args+=("--extra-vars" "install_cuda_drivers=false")
fi

# Check installation of dev package
if [ "$option_no_dev" = "true" ]; then
    ansible_args+=("--extra-vars" "install_devel=false")
    ansible_args+=("--extra-vars" "installation_type=ros-base")
else
    ansible_args+=("--extra-vars" "install_devel=true")
fi

# Load env
source "$SCRIPT_DIR/amd64.env"
if [ "$(uname -m)" = "aarch64" ]; then
    source "$SCRIPT_DIR/arm64.env"
fi

# Add env args
# shellcheck disable=SC2013
for env_name in $(sed -e "s/^\s*//" -e "/^#/d" -e "s/=.*//" <amd64.env); do
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

# Install pipx for ansible
if ! (python3 -m pipx --version >/dev/null 2>&1); then
    sudo apt-get -y update
    sudo apt-get -y install python3-pip python3-venv
    python3 -m pip install --user pipx
fi

# Install ansible
python3 -m pipx ensurepath
export PATH="${PIPX_BIN_DIR:=$HOME/.local/bin}:$PATH"
pipx install --include-deps --force "ansible==6.*"

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
