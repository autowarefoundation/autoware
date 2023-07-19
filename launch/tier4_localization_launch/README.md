# tier4_localization_launch

## Structure

![tier4_localization_launch](./localization_launch.drawio.svg)

## Package Dependencies

Please see `<exec_depend>` in `package.xml`.

## Usage

Include `localization.launch.xml` in other launch files as follows.

You can select which methods in localization to launch as `pose_estimator` or `twist_estimator` by specifying `pose_source` and `twist_source`.

In addition, you should provide parameter paths as `PACKAGE_param_path`. The list of parameter paths you should provide is written at the top of `localization.launch.xml`.

```xml
  <include file="$(find-pkg-share tier4_localization_launch)/launch/localization.launch.xml">
    <!-- Localization methods -->
    <arg name="pose_source" value="..."/>
    <arg name="twist_source" value="..."/>

    <!-- Parameter files -->
    <arg name="FOO_param_path" value="..."/>
    <arg name="BAR_param_path" value="..."/>
    ...
  </include>
```
