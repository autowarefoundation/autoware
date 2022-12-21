# tier4_perception_launch

## Structure

![tier4_perception_launch](./perception_launch.drawio.svg)

## Package Dependencies

Please see `<exec_depend>` in `package.xml`.

## Usage

You can include as follows in `*.launch.xml` to use `perception.launch.xml`.

Note that you should provide parameter paths as `PACKAGE_param_path`. The list of parameter paths you should provide is written at the top of `perception.launch.xml`.

```xml
  <include file="$(find-pkg-share tier4_perception_launch)/launch/perception.launch.xml">
    <!-- options for mode: camera_lidar_fusion, lidar, camera -->
    <arg name="mode" value="lidar" />

    <!-- Parameter files -->
    <arg name="FOO_param_path" value="..."/>
    <arg name="BAR_param_path" value="..."/>
    ...
  </include>
```
