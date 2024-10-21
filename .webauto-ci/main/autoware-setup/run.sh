
: "${WEBAUTO_CI_GITHUB_TOKEN:?is not set}"

sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(source /etc/os-release && echo "$UBUNTU_CODENAME") main" | sudo tee /etc/apt/sources.list.d/ros2.list >/dev/null
sudo -E apt-get -y update
sudo -E apt-get -y install usbutils # For kvaser

export GITHUB_TOKEN="$WEBAUTO_CI_GITHUB_TOKEN"
git config --global --add url."https://${GITHUB_TOKEN}:x-oauth-basic@github.com/".insteadOf "https://github.com/"
git config --global --add url."https://${GITHUB_TOKEN}:x-oauth-basic@github.com/".insteadOf "git@github.com:"

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
ansible-playbook "ansible/playbooks/local_dev_env.yaml" \
    "${ansible_args[@]}" \
    -e WORKSPACE_ROOT="$(pwd)" \
    -e install_devel="y" \
    --skip-tags vcs

git config --global --unset-all url."https://${GITHUB_TOKEN}:x-oauth-basic@github.com/".insteadOf
