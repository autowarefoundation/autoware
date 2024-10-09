# qt5ct_setup Ansible role

## Overview

The `qt5ct_setup` Ansible role automates the configuration of the `qt5ct` environment for Autoware.
It won't affect the system-wide configuration.

## Installation

Follow the instructions below to **install** or **update** the custom theme for `RViz2` in Autoware.

> **Important:** Both commands must be run when you want to update the theme.

```bash
cd ~/autoware # The root directory of the cloned repository
ansible-galaxy collection install -f -r "ansible-galaxy-requirements.yaml"
ansible-playbook autoware.dev_env.install_rviz_theme  --ask-become-pass
```

## How to use the custom theme in RViz2

To apply a custom theme to RViz2, you can use the `qt5ct` platform theme. Follow these steps to ensure that the `QT_QPA_PLATFORMTHEME` environment variable is set correctly for your RViz2 instance when used with Autoware.

### Manual setup for running RViz2

Before running RViz2 manually, set the `QT_QPA_PLATFORMTHEME` environment variable to `qt5ct`.
This ensures that the custom theme settings are applied.

```bash
export QT_QPA_PLATFORMTHEME=qt5ct
```

Then, start RViz2 as usual.

```bash
rviz2
```

### Automatic setup in Autoware

In Autoware, the `QT_QPA_PLATFORMTHEME` environment variable is automatically set within the main [autoware.launch.xml](https://github.com/autowarefoundation/autoware_launch/blob/main/autoware_launch/launch/autoware.launch.xml) file.
Therefore, you do not need to manually set this environment variable when launching Autoware.

In the `autoware.launch.xml` file, RViz2 is configured with the following `<node>` element:

```xml
<node
  pkg="rviz2"
  exec="rviz2"
  name="rviz2"
  output="screen"
  args="-d $(var rviz_config) -s $(find-pkg-share autoware_launch)/rviz/image/autoware.png"
  if="$(var rviz)"
  respawn="$(var rviz_respawn)">
  <env name="QT_QPA_PLATFORMTHEME" value="qt5ct"/>
</node>
```

This configuration automatically sets the `QT_QPA_PLATFORMTHEME` to `qt5ct` when RViz2 is launched as part of Autoware.
It also includes additional options such as respawn behavior and custom RViz2 configurations.
