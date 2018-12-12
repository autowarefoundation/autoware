# The `costmap_generator` Package

This package contains nodes to extract information from the LiDAR sensor, from the result of the perception module and/or from ADAS Maps.
The processed result is published as GridMaps (grid_map) and occupancy grids(nav_msgs).

---

## costmap_generator

This node reads PointCloud and/or DetectedObjectArray amd/or VectorMap and creates an occupancy grid and grid map based on the data contained in it.

The subscribed PointCloud topic should be a processed PointCloud with the ground previously removed.
The default input is set as the output PointCloud result from the node `euclidean_clustering` of the `lidar_tracker` package. However, another option might be the output of the `ray_ground_filter` or `ring_ground_filter`, both belonging to the `points_preprocessor` in the *Sensing* package.

The values contained within the OccupancyGrid range from 0-100, representing a probability percentage where 0 represents free space and 100 represents occupied space.

#### Input topics
`/points_no_ground` (sensor_msgs::PointCloud2) : from ray_ground_filter or compare map filter. It contains filtered points with the ground removed.

`/prediction/moving_predictor/objects` (autoware_msgs::DetectedObjectArray): predicted objects from naive_motion_predict.

`/vector_map`: from the VectorMap publisher. `/tf` to obtain the transform between the vector map(grid_frame) and the sensor(sensor_frame) .


#### Output topics
`/semantics/costmap` (grid_map::GridMap) is the output costmap, with values ranging from 0.0-1.0.

`/semantics/costmap_generator/occupancy_grid` (nav_msgs::OccupancyGrid) is the output OccupancyGrid, with values ranging from 0-100.

##### How to launch
It can be launched as follows:
 1. Using the Runtime Manager by clicking the `costmap_generator` checkbox under the *Semantics* section in the Computing tab.
 2. From a sourced terminal executing: `roslaunch costmap_generator costmap_generator.launch`.

##### Parameters available in roslaunch and rosrun

---

## Instruction Videos
