# The `costmap_generator` Package
## costmap_generator

This node reads `PointCloud` and/or `DetectedObjectArray` and creates an `OccupancyGrid` and `GridMap`. `VectorMap` is optional.

**You need to subscribe at least one of `PointCloud` or `DetectedObjectArray` to generate costmap.**




#### Input topics
`/points_no_ground` (sensor_msgs::PointCloud2) : from ray_ground_filter or compare map filter. It contains filtered points with the ground removed.

`/prediction/moving_predictor/objects` (autoware_msgs::DetectedObjectArray): predicted objects from naive_motion_predict.

`/vector_map`: from the VectorMap publisher. `/tf` to obtain the transform between the vector map(map_frame) and the sensor(sensor_frame) .


#### Output topics
`/semantics/costmap` (grid_map::GridMap) is the output costmap, with values ranging from 0.0-1.0.

`/semantics/costmap_generator/occupancy_grid` (nav_msgs::OccupancyGrid) is the output OccupancyGrid, with values ranging from 0-100.

##### How to launch
It can be launched as follows:
 1. Using the Runtime Manager by clicking the `costmap_generator` checkbox under the *Semantics* section in the Computing tab.
 2. From a sourced terminal executing: `roslaunch costmap_generator costmap_generator.launch`.

##### Parameters available in roslaunch and rosrun
* `use_objects` Whether if using `DetectedObjectArray` or not (default value: true).
* `use_points` Whether if using `PointCloud` or not (default value: true).
* `use_wayarea` Whether if using `Wayarea` from `VectorMap` or not (default value: true).
* `objects_input` Input topic for `autoware_msgs::DetectedObjectArray` (default value: /prediction/moving_predictor/objects).
* `points_input` Input topic for sensor_msgs::PointCloud2 (default value: points_no_ground).
* `lidar_frame` Lidar sensor coordinate. Cost is calculated based on this coordinate (default value: velodyne).
* `map_frame` VectorMap's coordinate. (default value: map).
* `grid_min_value` Minimum cost for gridmap (default value: 0.0).
* `grid_max_value` Maximum cost for gridmap (default value: 1.0).
* `grid_resolution` Resolution for gridmap (default value: 0.2).
* `grid_length_x` Size of gridmap for x direction (default value: 50).
* `grid_length_y` Size of gridmap for y direction (default value: 30).
* `grid_position_x` Offset from coordinate in x direction (default value: 20).
* `grid_position_y` Offset from coordinate in y direction (default value:  0).
* `maximum_lidar_height_thres` Maximum height threshold for pointcloud data (default value:  0.3).
* `minimum_lidar_height_thres` Minimum height threshold for pointcloud data (default value:  -2.2).
* `expand_rectangle_size` Expand object's rectangle with this value (default value: 1).
* `size_of_expansion_kernel` Kernel size for blurring effect on object's costmap (default value: 9).

---

## Instruction Videos

[![](https://img.youtube.com/vi/f7kSVJ23Mtw/0.jpg)](https://www.youtube.com/watch?v=f7kSVJ23Mtw)
