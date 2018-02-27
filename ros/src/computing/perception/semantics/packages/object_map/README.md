# The `object_map` package

This package contains nodes to extract information from the LiDAR sensor and from ADAS Maps. 
The processed result is published as GridMaps(grid_map) and/or Occupancy Grids(nav_msgs).

---

## Nodes in the package

### points2costmap

This package reads PointCloud data and creates an occupancy grid based on the data contained in it.

The subscribed PointCloud topic should be a processed PointCloud with the ground previously removed.
The default input is set to the output PointCloud result from the node `euclidean_clustering` from the `lidar_tracker` package. However, other option might be `ray_ground_filter` or `ring_ground_filter`, both belong to the `points_preprocessor` in the *Sensing* Package.

The values contained in the OccupancyGrid range from 0-100 in probability percentage. 
Where 0 represents free space and 100 occupied.

#### Input topics
Default : `/points_lanes` (PointCloud) from euclidean cluster. It contains points without ground and filtered. Other possible option `/points_no_ground`

#### Output topics
`/realtime_cost_map` (nav_msgs::OccupancyGrid). Resulting Occupancy grid. Values ranging from 0-100.

##### How to lunch
It can be launched 
 1. Runtime Manager clicking the `points2costmap` checkbox under the *Semantics* section in the Computing tab.
 2. From a sourced terminal executing: `roslaunch object_map points2costmap.launch`

##### Parameters available in roslaunch and rosrun
* `resolution` Defines the equivalent value of a cell in the grid in meters. Smaller values have better accuracy at the memory and computing cost (Default value: 0.1)
* `cell_width` Represents the width in meters of the occupancy grid around the origin of the pointcloud (Default: 150)
* `cell_height` Represents the height in meters of the occupancy grid around the origin of the pointcloud (Default: 150)
* `offset_x` Indicates if the center of the Occupancy will be shifted by this distance, to the left(-) or right(+) (Default: 25)
* `offset_y` Indicates if the center of the Occupancy will be shifted by this distance, to the back(-) or front(+)  (Default: 0)
* `offset_z` Indicates if the center of the Occupancy will be shifted by this distance, below(-) or above(+) (Default: 0)
* `points_topic` PointCloud topic source

---

### laserscan2costmap

It performs the same function as `points2costmap` but instead it uses 2D LaserScan messages to generate the OccupancyGrid.

#### Input topics
Default : `/scan` (sensor_msgs::LaserScan) from vscan.

#### Output topics
`/ring_ogm` (nav_msgs::OccupancyGrid). Resulting Occupancy grid. Values ranging from 0-100.

##### How to lunch
It can be launched 
 1. Runtime Manager clicking the `laserscan2costmap` checkbox under the *Semantics* section in the Computing tab.
 2. From a sourced terminal executing: `roslaunch object_map laserscan2costmap.launch`
 
 ##### Parameters available in roslaunch and rosrun
 * `resolution` Defines the equivalent value of a cell in the grid in meters. Smaller values have better accuracy at the memory and computing cost (Default value: 0.1)
 * `cell_width` Represents the width in meters of the occupancy grid around the origin of the pointcloud (Default: 150)
 * `cell_height` Represents the height in meters of the occupancy grid around the origin of the pointcloud (Default: 150)
 * `offset_x` Indicates if the center of the Occupancy will be shifted by this distance, to the left(-) or right(+) (Default: 25)
 * `offset_y` Indicates if the center of the Occupancy will be shifted by this distance, to the back(-) or front(+)  (Default: 0)
 * `offset_z` Indicates if the center of the Occupancy will be shifted by this distance, below(-) or above(+) (Default: 0)
 * `scan_topic` PointCloud topic source

---

### wayarea2grid
ADAS Maps used in Autoware might contain the definition of the road areas, this is useful to compute whether a region is drivable or not.
This node reads the data from the ADAS Map (VectorMap) and extracts the 3D positions of the road regions. It projects them to an OccupancyGrid and sets the value to `128` if the area is **road** and `255` when it is not.
This node uses 8-bit bitmaps (grid_map), therefore the previous values.

#### Input topics
`/vector_map` (vector_map_msgs::WayArea) from VectorMap publisher.
`/tf` To obtain the transform between the vector map(grid_frame) and the sensor(sensor_frame) .

#### Output topics
`/grid_map_wayarea` (grid_map::GridMap). Resulting GridMap. Values ranging from 0-255.
`/occupancy_wayarea` (nav_msgs::OccupancyGrid). Resulting Occupancy grid. Values ranging from 0-255.

##### How to lunch
It can be launched 
 1. Runtime Manager clicking the `wayarea2grid` checkbox under the *Semantics* section in the Computing tab.
 2. From a sourced terminal executing: `roslaunch object_map wayarea2grid.launch`

##### Parameters available in roslaunch and rosrun
* `sensor_frame` Defines the coordinate frame of the vehicle origin (Default value: velodyne)
* `grid_frame` Defines the coordinate frame of the map (Default: map)
* `grid_resolution` Defines the equivalent value of a cell in the grid in meters. Smaller values have better accuracy at the memory and computing cost (Default: 0.2)
* `grid_length_x` Represents the width in meters of the occupancy grid around the origin of the pointcloud (Default: 25)
* `grid_length_y` Represents the height in meters of the occupancy grid around the origin of the pointcloud  (Default: 0)
* `grid_position_x` Indicates if the center of the Occupancy will be shifted by this distance, left(-) or right(+) (Default: 0)
* `grid_position_y` Indicates if the center of the Occupancy will be shifted by this distance, back(-) or front(+) (Default: 0)

### grid_map_filter
This node can combine data from the sensor, map, perception data as well as other OccupancyGrids. It generates OccupancyGrids with useful data for navigation purposes.
It publishes three different layers. Details are found in the output topics section below.

#### Input topics
`/vector_map` (vector_map_msgs::WayArea) from VectorMap publisher.
`/tf` To obtain the transform between the vector map(grid_frame) and the sensor(sensor_frame).
`/realtime_cost_map` (nav_msgs::OccupancyGrid). Calculated by `points2costmap` in this package.

#### Output topics
It also publishes:
`/filtered_grid_map`(grid_map::GridMap) which contains 3 layers and are also published as regular Occupancy grid messages.

`distance_transform` (nav_msgs::OccupancyGrid). It applies a distance transform to the occupancy grid obtained from the `/realtime_cost_map`. This allows to obtain a gradient of probabilities surrounding the obstacles PointCloud.
`dist_wayarea` (nav_msgs::OccupancyGrid). Contains a mixture between the distance transform and the road data, as described in the `wayarea2grid` node.
`circle` (nav_msgs::OccupancyGrid). Draws circle surrounding the `/realtime_cost_map`.

The output topics are configured as described in the `grid_map` package, the confiration file is inside the `config` folder of the package.

##### How to lunch
It can be launched 
 1. Runtime Manager clicking the `grid_map_filter` checkbox under the *Semantics* section in the Computing tab.
 2. From a sourced terminal executing: `roslaunch object_map grid_map_filter.launch`
 

##### Parameters available in roslaunch and rosrun
 * `map_frame` Coordinate system of the realtime costmap (Default value: map).
 * `map_topic` Topic where the realtime costmap is being published (Default: /realtime_cost_map).
 * `dist_transform_distance` Maximum distance to calculate the distance transform, in meters (Default: 2.0).
 * `use_wayarea` Indicates whether or not to use the road regions to filter the cost map (Default: true).
 * `use_fill_circle` Enables or disables the generation of the circle layer (Default: true).
 * `fill_circle_cost_threshold` Minimum cost value threshold value to decide if a circle will be drawn (Default: 20)
 * `circle_radius` Radius of the circle, in meters (Default: 1.7).

---

## Instruction Videos

### grid_map_filter

[![](https://img.youtube.com/vi/bl-nG8Zv-C0/0.jpg)](https://www.youtube.com/watch?v=bl-nG8Zv-C0)

### wayarea2grid

[![](https://img.youtube.com/vi/UkcO4V-0NOw/0.jpg)](https://www.youtube.com/watch?v=UkcO4V-0NOw)
