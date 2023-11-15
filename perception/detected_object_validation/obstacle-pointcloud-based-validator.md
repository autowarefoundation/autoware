# obstacle pointcloud based validator

## Inner-workings / Algorithms

If the number of obstacle point groups in the DetectedObjects is small, it is considered a false positive and removed.
The obstacle point cloud can be a point cloud after compare map filtering or a ground filtered point cloud.

![debug sample image](image/obstacle_pointcloud_based_validator/debug_image.gif)

In the debug image above, the red DetectedObject is the validated object. The blue object is the deleted object.

## Inputs / Outputs

### Input

| Name                          | Type                                                  | Description                             |
| ----------------------------- | ----------------------------------------------------- | --------------------------------------- |
| `~/input/detected_objects`    | `autoware_auto_perception_msgs::msg::DetectedObjects` | DetectedObjects                         |
| `~/input/obstacle_pointcloud` | `sensor_msgs::msg::PointCloud2`                       | Obstacle point cloud of dynamic objects |

### Output

| Name               | Type                                                  | Description               |
| ------------------ | ----------------------------------------------------- | ------------------------- |
| `~/output/objects` | `autoware_auto_perception_msgs::msg::DetectedObjects` | validated DetectedObjects |

## Parameters

| Name                            | Type  | Description                                                                                                                                                                |
| ------------------------------- | ----- | -------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `using_2d_validator`            | bool  | The xy-plane projected (2D) obstacle point clouds will be used for validation                                                                                              |
| `min_points_num`                | int   | The minimum number of obstacle point clouds in DetectedObjects                                                                                                             |
| `max_points_num`                | int   | The max number of obstacle point clouds in DetectedObjects                                                                                                                 |
| `min_points_and_distance_ratio` | float | Threshold value of the number of point clouds per object when the distance from baselink is 1m, because the number of point clouds varies with the distance from baselink. |
| `enable_debugger`               | bool  | Whether to create debug topics or not?                                                                                                                                     |

## Assumptions / Known limits

Currently, only represented objects as BoundingBox or Cylinder are supported.
