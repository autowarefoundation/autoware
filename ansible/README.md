# Ansible Collection - autoware.dev_env

This collection contains the playbooks to set up the development environment for Autoware.

## Set up a development environment

### Ansible installation

```bash
# Remove apt installed ansible (In Ubuntu 22.04, ansible the version is old)
sudo apt-get purge ansible

# Install pipx
sudo apt-get -y update
sudo apt-get -y install pipx

# Add pipx to the system PATH
python3 -m pipx ensurepath

# Install ansible
pipx install --include-deps --force "ansible==6.*"
```

### Install ansible collections

This step should be repeated when a new playbook is added.

```bash
cd ~/autoware # The root directory of the cloned repository
ansible-galaxy collection install -f -r "ansible-galaxy-requirements.yaml"
```
