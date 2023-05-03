# tier4_control_launch

## Structure

![tier4_control_launch](./control_launch.drawio.svg)

## Package Dependencies

Please see `<exec_depend>` in `package.xml`.

## Usage

You can include as follows in `*.launch.xml` to use `control.launch.py`.

Note that you should provide parameter paths as `PACKAGE_param_path`. The list of parameter paths you should provide is written at the top of `planning.launch.xml`.

```xml
<include file="$(find-pkg-share tier4_control_launch)/launch/control.launch.py">
  <!-- options for lateral_controller_mode: mpc_follower, pure_pursuit -->
  <!-- Parameter files -->
  <arg name="FOO_NODE_param_path" value="..."/>
  <arg name="BAR_NODE_param_path" value="..."/>
  ...
  <arg name="lateral_controller_mode" value="mpc_follower" />
</include>
```

## Notes

For reducing processing load, we use the [Component](https://docs.ros.org/en/galactic/Concepts/About-Composition.html) feature in ROS 2 (similar to Nodelet in ROS 1 )
