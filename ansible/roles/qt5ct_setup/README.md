# qt5ct_setup Ansible role

## Overview

The `qt5ct_setup` Ansible role automates the configuration of the `qt5ct` environment for Autoware. It ensures `qt5ct` is installed, configures the `qt5ct` settings, updates the QSS file paths, and ensures the correct Autoware directory is used.

## Installation

### Install ansible collections

```bash
cd ~/autoware # The root directory of the cloned repository
ansible-galaxy collection install -f -r "ansible-galaxy-requirements.yaml"
```

This step should be repeated when a new playbook is added.

### Run the playbook

```bash
ansible-playbook autoware.dev_env.install_rviz_theme  --ask-become-pass
```

This will download and extract the artifacts to the specified directory and validate the checksums.

## Usage

Include the `qt5ct_setup` role in your playbook. Ensure your playbook includes the `autoware` directory and the required files (`base-qt5ct.conf` and `autoware.qss`) in the appropriate locations.
