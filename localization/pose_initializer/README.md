# pose_initializer

## Purpose

`pose_initializer` is the package to send an initial pose to `ekf_localizer`.
It receives roughly estimated initial pose from GNSS/user.
Passing the pose to `ndt_scan_matcher`, and it gets a calculated ego pose from `ndt_scan_matcher` via service.
Finally, it publishes the initial pose to `ekf_localizer`.

## Input / Output

### Input topics

| Name                                 | Type                                          | Description            |
| ------------------------------------ | --------------------------------------------- | ---------------------- |
| `/initialpose`                       | geometry_msgs::msg::PoseWithCovarianceStamped | initial pose from rviz |
| `/sensing/gnss/pose_with_covariance` | geometry_msgs::msg::PoseWithCovarianceStamped | pose from gnss         |
| `/map/pointcloud_map`                | sensor_msgs::msg::PointCloud2                 | pointcloud map         |

### Output topics

| Name             | Type                                          | Description                 |
| ---------------- | --------------------------------------------- | --------------------------- |
| `/initialpose3d` | geometry_msgs::msg::PoseWithCovarianceStamped | calculated initial ego pose |
