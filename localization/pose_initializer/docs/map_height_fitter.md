# map_height_fitter

## Purpose

This node provides a service to fit a pose height to a map.
Use this service as preprocessing for `pose_initializer` when using a initial poses with inaccurate height such as RViz and GNSS.
This service replaces the Z value of the input pose with the lowest point of the map point cloud within a cylinder of XY-radius.
If no point is found in this range, returns the input pose without changes.

Note that this package supports partial map loading interface, which is disabled by default. The interface is intended to be enabled when
the pointcloud map is too large to handle. By using the interface, the node will request for a partial map around the requested position
instead of loading whole map by subscription interface. To use this interface,

1. Set `enable_partial_map_load` in this node to `true`
2. Set `enable_partial_load` in `pointcloud_map_loader` to `true`

## Interfaces

### Parameters

| Name                      | Type | Description                                                                           |       |
| ------------------------- | ---- | ------------------------------------------------------------------------------------- | ----- |
| `enable_partial_map_load` | bool | If true, use partial map load interface. If false, use topic subscription interaface. | false |

### Services

| Name                                | Type                                                    | Description          |
| ----------------------------------- | ------------------------------------------------------- | -------------------- |
| `/localization/util/fit_map_height` | tier4_localization_msgs::srv::PoseWithCovarianceStamped | pose fitting service |

### Subscriptions

| Name                  | Type                          | Description    |
| --------------------- | ----------------------------- | -------------- |
| `/map/pointcloud_map` | sensor_msgs::msg::PointCloud2 | pointcloud map |

### Clients

| Name                              | Type                                            | Description                                  |
| --------------------------------- | ----------------------------------------------- | -------------------------------------------- |
| `/map/get_partial_pointcloud_map` | autoware_map_msgs::srv::GetPartialPointCloudMap | client for requesting partial pointcloud map |
