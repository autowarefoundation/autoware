# Autoware Global Parameter Loader

This package is to set common ROS parameters to each node.

## Usage

Add the following lines to the launch file of the node in which you want to get global parameters.

```xml
<!-- Global parameters -->
  <include file="$(find-pkg-share autoware_global_parameter_loader)/launch/global_params.launch.py">
    <arg name="vehicle_model" value="$(var vehicle_model)"/>
  </include>
```

## Assumptions / Known limits

Currently only vehicle_info is loaded by this launcher.
