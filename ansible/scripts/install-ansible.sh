#!/usr/bin/env bash
# Install Ansible (and its OS prerequisites) so that the playbooks under
# autoware/ansible/ can be run.
#
# Does NOT install Galaxy collections or run any playbook. After this script,
# run from the repo root:
#   ansible-galaxy collection install -f -r ansible-galaxy-requirements.yaml
#   ansible-playbook autoware.dev_env.install_dev_env [--skip-tags nvidia ...]

set -euo pipefail

if ! command -v sudo >/dev/null 2>&1; then
    apt-get -y update
    apt-get -y install sudo
fi

sudo apt-get -y update
sudo apt-get -y install git python3-pip python3-venv pipx

python3 -m pipx ensurepath
export PATH="${PIPX_BIN_DIR:-$HOME/.local/bin}:$PATH"

pipx install --include-deps --force "ansible==10.*"

echo "ansible installed: $(ansible --version | head -n1)"
