# autoware_occupancy_grid_map_outlier_filter

## Purpose

This node is an outlier filter based on a occupancy grid map.
Depending on the implementation of occupancy grid map, it can be called an outlier filter in time series, since the occupancy grid map expresses the occupancy probabilities in time series.

## Inner-workings / Algorithms

1. Use the occupancy grid map to separate point clouds into those with low occupancy probability and those with high occupancy probability.

2. The point clouds that belong to the low occupancy probability are not necessarily outliers. In particular, the top of the moving object tends to belong to the low occupancy probability. Therefore, if `use_radius_search_2d_filter` is true, then apply an radius search 2d outlier filter to the point cloud that is determined to have a low occupancy probability.
   1. For each low occupancy probability point, determine the outlier from the radius (`radius_search_2d_filter/search_radius`) and the number of point clouds. In this case, the point cloud to be referenced is not only low occupancy probability points, but all point cloud including high occupancy probability points.
   2. The number of point clouds can be multiplied by `radius_search_2d_filter/min_points_and_distance_ratio` and distance from base link. However, the minimum and maximum number of point clouds is limited.

The following video is a sample. Yellow points are high occupancy probability, green points are low occupancy probability which is not an outlier, and red points are outliers. At around 0:15 and 1:16 in the first video, a bird crosses the road, but it is considered as an outlier.

- [movie1](https://www.youtube.com/watch?v=hEVv0LaTpP8)
- [movie2](https://www.youtube.com/watch?v=VaHs1CdLcD0)

![occupancy_grid_map_outlier_filter](./image/occupancy_grid_map_outlier_filter.drawio.svg)

## Inputs / Outputs

### Input

| Name                         | Type                      | Description                                                                                |
| ---------------------------- | ------------------------- | ------------------------------------------------------------------------------------------ |
| `~/input/pointcloud`         | `sensor_msgs/PointCloud2` | Obstacle point cloud with ground removed.                                                  |
| `~/input/occupancy_grid_map` | `nav_msgs/OccupancyGrid`  | A map in which the probability of the presence of an obstacle is occupancy probability map |

### Output

| Name                                        | Type                      | Description                                                                                                                  |
| ------------------------------------------- | ------------------------- | ---------------------------------------------------------------------------------------------------------------------------- |
| `~/output/pointcloud`                       | `sensor_msgs/PointCloud2` | Point cloud with outliers removed. trajectory                                                                                |
| `~/output/debug/outlier/pointcloud`         | `sensor_msgs/PointCloud2` | Point clouds removed as outliers.                                                                                            |
| `~/output/debug/low_confidence/pointcloud`  | `sensor_msgs/PointCloud2` | Point clouds that had a low probability of occupancy in the occupancy grid map. However, it is not considered as an outlier. |
| `~/output/debug/high_confidence/pointcloud` | `sensor_msgs/PointCloud2` | Point clouds that had a high probability of occupancy in the occupancy grid map. trajectory                                  |

## Parameters

{{ json_to_markdown("perception/occupancy_grid_map_outlier_filter/schema/occupancy_grid_map_outlier_filter.schema.json") }}

## Assumptions / Known limits

## (Optional) Error detection and handling

## (Optional) Performance characterization

## (Optional) References/External links

## (Optional) Future extensions / Unimplemented parts
