# tier4_autoware_api_launch

## Description

This package contains launch files that run nodes to convert Autoware internal topics into consistent API used by external software (e.g., fleet management system, simulator).

## Package Dependencies

Please see `<exec_depend>` in `package.xml`.

## Usage

You can include as follows in `*.launch.xml` to use `control.launch.py`.

```xml
  <include file="$(find-pkg-share tier4_autoware_api_launch)/launch/autoware_api.launch.xml">
    <arg name="init_simulator_pose" value="true"/>
    <arg name="init_localization_pose" value="false"/>
  </include>
```

## Notes

For reducing processing load, we use the [Component](https://docs.ros.org/en/galactic/Concepts/About-Composition.html) feature in ROS2 (similar to Nodelet in ROS1 )
