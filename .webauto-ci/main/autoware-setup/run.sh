#!/bin/bash -e

ansible_args=()
ansible_args+=("--extra-vars" "prompt_install_nvidia=y")
ansible_args+=("--extra-vars" "prompt_download_artifacts=y")
ansible_args+=("--extra-vars" "data_dir=$HOME/autoware_data")
ansible_args+=("--extra-vars" "ros2_installation_type=ros-base")
ansible_args+=("--extra-vars" "install_devel=false")

# read amd64 env file and expand ansible arguments
source 'amd64.env'
while read -r env_name; do
    ansible_args+=("--extra-vars" "${env_name}=${!env_name}")
done < <(sed "s/=.*//" <amd64.env)

ansible-galaxy collection install -f -r "ansible-galaxy-requirements.yaml"
ansible-playbook "ansible/playbooks/universe.yaml" \
    "${ansible_args[@]}" \
    -e WORKSPACE_ROOT="$(pwd)" \
    --skip-tags vcs
