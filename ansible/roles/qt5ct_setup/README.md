# qt5ct_setup Ansible role

## Overview

The `qt5ct_setup` Ansible role automates the configuration of the `qt5ct` environment for Autoware. It ensures `qt5ct` is installed, configures the `qt5ct` settings, updates the QSS file paths, and ensures the correct Autoware directory is used.

## Role variables

This role does not require any external variables. It dynamically determines the Autoware directory within the user's home directory.

## Tasks

### 1. Install qt5ct

Installs the `qt5ct` package if it is not already present on the system.

### 2. Find Autoware directory

Searches for directories named `autoware` within the user's home directory and verifies they contain the `ansible` folder to ensure it's the correct Autoware directory.

### 3. Comment out `QT_QPA_PLATFORMTHEME` export line

Finds and comments out any `QT_QPA_PLATFORMTHEME=qt5ct` export lines in the `/etc/X11/Xsession.d` directory to prevent conflicts with other qt applications.

### 4. Ensure qt5ct configuration directory exists

Creates the `qt5ct` configuration directory (`~/.config/qt5ct`) if it does not exist.

### 5. Copy base `qt5ct.conf`

Copies a base `qt5ct.conf` configuration file if it does not already exist in the `~/.config/qt5ct` directory.

### 6. Append new stylesheet

Appends the new stylesheet path to the `qt5ct.conf` file, ensuring it does not overwrite existing stylesheets.

### 7. Set color scheme and style

Ensures the color scheme is set to `darker.conf` and the style is set to `Fusion` in the `qt5ct.conf` file.

### 8. Update QSS file paths

Updates the paths in the `autoware.qss` file to use absolute paths for icons.

## Usage

Include the `qt5ct_setup` role in your playbook. Ensure your playbook includes the `autoware` directory and the required files (`base-qt5ct.conf` and `autoware.qss`) in the appropriate locations.

### Example playbook

```yaml
- name: Setup qt5ct for Autoware
  hosts: localhost
  roles:
    - qt5ct_setup
```

## File structure

```
qt5ct_setup/
├── files/
│   ├── base-qt5ct.conf
│   ├── autoware.qss
│   └── autoware-rviz-icons/
├── tasks/
│   └── main.yml
└── README.md
```
