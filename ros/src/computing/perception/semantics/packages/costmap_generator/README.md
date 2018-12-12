# The `costmap_generator` Package
## costmap_generator

This node reads PointCloud and/or DetectedObjectArray and/or VectorMap and creates an occupancy grid and grid map.

You need to specify at least one of three topics to generate costmap.

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
