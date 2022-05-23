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
        option_yes=true
        ;;
    -v)
        option_verbose=true
        ;;
    --no-nvidia)
        option_no_nvidia=true
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

# Check NVIDIA Installation
if [ "$option_no_nvidia" = "true" ]; then
    ansible_args+=("--extra-vars" "install_nvidia=n")
elif [ "$option_yes" = "true" ]; then
    ansible_args+=("--extra-vars" "install_nvidia=y")
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

# Install pip for ansible
if ! (command -v pip3 >/dev/null 2>&1); then
    sudo apt-get -y update
    sudo apt-get -y install python3-pip
fi

# Install ansible
ansible_version=$(pip3 list | grep -oP "^ansible\s+\K([0-9]+)" || true)
if [ "$ansible_version" != "5" ]; then
    sudo apt-get -y purge ansible
    pip3 install -U "ansible==5.*"
fi

# For Python packages installed with user privileges
export PATH="$HOME/.local/bin:$PATH"

# Install ansible collections
ansible-galaxy collection install -f -r "$SCRIPT_DIR/ansible-galaxy-requirements.yaml"

# Run ansible
echo -e "\e[36m Run ansible-playbook" "$target_playbook" "${ansible_args[@]}" "\e[m"
if ansible-playbook "$target_playbook" "${ansible_args[@]}"; then
    echo -e "\e[32mCompleted.\e[0m"
    exit 0
else
    echo -e "\e[31mFailed.\e[0m"
    exit 1
fi
