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

| Name                 | Type  | Description                                                                  |
| -------------------- | ----- | ---------------------------------------------------------------------------- |
| `min_pointcloud_num` | float | Threshold for the minimum number of obstacle point clouds in DetectedObjects |
| `enable_debugger`    | bool  | Whether to create debug topics or not?                                       |

## Assumptions / Known limits

Currently, only represented objects as BoundingBox or Cylinder are supported.
