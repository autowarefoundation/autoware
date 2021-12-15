# tier4_perception_launch

## Structure

![tier4_perception_launch](./perception_launch.drawio.svg)

## Package Dependencies

Please see `<exec_depend>` in `package.xml`.

## Usage

You can include as follows in `*.launch.xml` to use `perception.launch.xml`.

```xml
  <include file="$(find-pkg-share tier4_perception_launch)/launch/perception.launch.xml">
    <!-- options for mode: camera_lidar_fusion, lidar, camera -->
    <arg name="mode" value="lidar" />
  </include>
```
