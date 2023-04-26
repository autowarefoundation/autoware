# map4_localization_launch

This localization launch package is a sample package to introduce eagleye into autoware.

Eagleye provides a cost-effective alternative to LiDAR and point cloud-based localization by using low-cost GNSS and IMU sensors to provide vehicle position and orientation.

Autoware users will be able to choose between their existing LiDAR and point cloud-based localization stacks or GNSS/IMU-based eagleye localizer, depending on their specific needs and operating environment.

In addition to the LiDAR-based solution provided by `tier4_localization_launch` (shown in the first row of the following table), there are two ways to utilize eagleye in `map4_localization_launch` with the autoware localization stack.

| localization launch                                             | twist estimator                   | pose estimator                    |
| --------------------------------------------------------------- | --------------------------------- | --------------------------------- |
| tier4_localization_launch                                       | gyro_odometry                     | ndt_scan_matcher                  |
| map4_localization_launch/eagleye_twist_localization_launch      | eagleye_rt(gyro/odom/gnss fusion) | ndt_scan_matcher                  |
| map4_localization_launch/eagleye_pose_twist_localization_launch | eagleye_rt(gyro/odom/gnss fusion) | eagleye_rt(gyro/odom/gnss fusion) |

## Structure

- eagleye_twist_localization_launch

![map4_localization_launch/eagleye_twist_localization_launch](./eagleye_twist_localization_launch/localization_launch.drawio.svg)

- eagleye_pose_twist_localization_launch

![map4_localization_launch/eagleye_pose_twist_localization_launch](./eagleye_pose_twist_localization_launch/localization_launch.drawio.svg)

## Package Dependencies

Please see `<exec_depend>` in `package.xml`.

## Usage

Include `localization.launch.xml` in other launch files as follows.

Note that you should provide parameter paths as `PACKAGE_param_path`. The list of parameter paths you should provide is written at the top of `localization.launch.xml`.

```xml
  <include file="$(find-pkg-share map4_localization_launch)/eagleye_twist_localization_launch/localization.launch.xml">
    <!-- Parameter files -->
    <arg name="FOO_param_path" value="..."/>
    <arg name="BAR_param_path" value="..."/>
    ...
  </include>
```
