# outlier_filter

## Purpose

The `outlier_filter` is a node that removes points caused by hardware problems, rain drops and small insects as a noise.

## Inner-workings / Algorithms

### Dual Return Outlier Filter

WIP

### Radius Search 2d Outlier Filter [1]

WIP

### Ring Outlier Filter

WIP

### Voxel Grid Outlier Filter

WIP

## Inputs / Outputs

### Input

| Name                 | Type                      | Description                               |
| -------------------- | ------------------------- | ----------------------------------------- |
| `~/input/pointcloud` | `sensor_msgs/PointCloud2` | Obstacle point cloud with ground removed. |

### Output

| Name                                | Type                      | Description                                   |
| ----------------------------------- | ------------------------- | --------------------------------------------- |
| `~/output/pointcloud`               | `sensor_msgs/PointCloud2` | Point cloud with outliers removed. trajectory |
| `~/output/debug/outlier/pointcloud` | `sensor_msgs/PointCloud2` | Point clouds removed as outliers.             |

## Parameters

| Name                                                    | Type   | Description                                                                                                                                                                |
| ------------------------------------------------------- | ------ | -------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `map_frame`                                             | string | map frame id                                                                                                                                                               |
| `base_link_frame`                                       | string | base link frame id                                                                                                                                                         |
| `enable_debugger`                                       | bool   | Whether to output the point cloud for debugging.                                                                                                                           |
| `radius_search_2d_filter/search_radius`                 | float  | Radius when calculating the density                                                                                                                                        |
| `radius_search_2d_filter/min_points_and_distance_ratio` | float  | Threshold value of the number of point clouds per radius when the distance from baselink is 1m, because the number of point clouds varies with the distance from baselink. |
| `radius_search_2d_filter/min_points`                    | int    | Minimum number of point clouds per radius                                                                                                                                  |
| `radius_search_2d_filter/max_points`                    | int    | Maximum number of point clouds per radius                                                                                                                                  |

## Assumptions / Known limits

## (Optional) Error detection and handling

## (Optional) Performance characterization

## (Optional) References/External links

[1] <https://pcl.readthedocs.io/projects/tutorials/en/latest/remove_outliers.html>

## (Optional) Future extensions / Unimplemented parts
