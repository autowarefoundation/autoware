# map_height_fitter

## Purpose

This node provides a service to fit a pose height to a map.
Use this service as preprocessing for `pose_initializer` when using a initial poses with inaccurate height such as RViz and GNSS.
This service replaces the Z value of the input pose with the lowest point of the map point cloud within a cylinder of XY-radius.
If no point is found in this range, returns the input pose without changes.

## Interfaces

### Services

| Name                                | Type                                                    | Description          |
| ----------------------------------- | ------------------------------------------------------- | -------------------- |
| `/localization/util/fit_map_height` | tier4_localization_msgs::srv::PoseWithCovarianceStamped | pose fitting service |

### Subscriptions

| Name                  | Type                          | Description    |
| --------------------- | ----------------------------- | -------------- |
| `/map/pointcloud_map` | sensor_msgs::msg::PointCloud2 | pointcloud map |
