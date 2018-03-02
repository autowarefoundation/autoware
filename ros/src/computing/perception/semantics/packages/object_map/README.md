# The `object_map` Package

This package contains nodes to extract information from the LiDAR sensor and from ADAS Maps.
The processed result is published as GridMaps (grid_map) and/or occupancy grids(nav_msgs).

---

## Nodes in the Package

### points2costmap

This node reads PointCloud data and creates an occupancy grid based on the data contained in it.

The subscribed PointCloud topic should be a processed PointCloud with the ground previously removed.
The default input is set as the output PointCloud result from the node `euclidean_clustering` of the `lidar_tracker` package. However, another option might be the output of the `ray_ground_filter` or `ring_ground_filter`, both belonging to the `points_preprocessor` in the *Sensing* package.

The values contained within the OccupancyGrid range from 0-100, representing a probability percentage where 0 represents free space and 100 represents occupied space.

#### Input topics
Default: `/points_lanes` (PointCloud) from euclidean cluster. It contains filtered points with the ground removed. Another possible option is `/points_no_ground`.

#### Output topics
`/realtime_cost_map` (nav_msgs::OccupancyGrid) is the output OccupancyGrid, with values ranging from 0-100.

##### How to launch
It can be launched as follows:
 1. Using the Runtime Manager by clicking the `points2costmap` checkbox under the *Semantics* section in the Computing tab.
 2. From a sourced terminal executing: `roslaunch object_map points2costmap.launch`.

##### Parameters available in roslaunch and rosrun
* `resolution` defines the equivalent value of a cell in the grid in meters. Smaller values result in better accuracy at the expense of memory and computing cost (default value: 0.1).
* `cell_width` represents the width in meters of the OccupancyGrid around the origin of the PointCloud (default: 150).
* `cell_height` represents the height in meters of the OccupancyGrid around the origin of the PointCloud (default: 150).
* `offset_x` indicates if the center of the OccupancyGrid will be shifted by this distance, to the left(-) or right(+) (default: 25).
* `offset_y` indicates if the center of the OccupancyGrid will be shifted by this distance, to the back(-) or front(+)  (default: 0).
* `offset_z` indicates if the center of the OccupancyGrid will be shifted by this distance, below(-) or above(+) (dDefault: 0).
* `points_topic` is the PointCloud topic source.

---

### laserscan2costmap

This node performs the same function as `points2costmap` but uses 2D LaserScan messages to generate the OccupancyGrid.

#### Input topics
Default: `/scan` (sensor_msgs::LaserScan) from vscan.

#### Output topics
`/ring_ogm` (nav_msgs::OccupancyGrid) is the output OccupancyGrid with values ranging from 0-100.

##### How to launch
It can be launched as follows:
 1. Using the Runtime Manager by clicking the `laserscan2costmap` checkbox under the *Semantics* section in the Computing tab.
 2. From a sourced terminal by executing: `roslaunch object_map laserscan2costmap.launch`.

##### Parameters available in roslaunch and rosrun
 * `resolution` defines the equivalent value of a cell in the grid in meters. Smaller values result in better accuracy at the expense of memory and computing cost (default value: 0.1).
 * `cell_width` represents the width in meters of the OccupancyGrid around the origin of the PointCloud (default: 150).
 * `cell_height` represents the height in meters of the OccupancyGrid around the origin of the PointCloud (default: 150).
 * `offset_x` indicates if the center of the OccupancyGrid will be shifted by this distance, to the left(-) or right(+) (default: 25).
 * `offset_y` indicates if the center of the OccupancyGrid will be shifted by this distance, to the back(-) or front(+)  (default: 0).
 * `offset_z` indicates if the center of the OccupancyGrid will be shifted by this distance, below(-) or above(+) (default: 0).
 * `scan_topic` is the PointCloud topic source.

---

### wayarea2grid

The ADAS Maps used in Autoware may contain the definition of the road areas, which is useful for computing whether a region is drivable or not.
This node reads the data from an ADAS Map (VectorMap) and extracts the 3D positions of the road regions. It projects them into an OccupancyGrid and sets the value to `128` if the area is **road** and `255` if it is not. These values are used because this node uses 8-bit bitmaps (grid_map).

#### Input topics
`/vector_map` (vector_map_msgs::WayArea) from the VectorMap publisher.
`/tf` to obtain the transform between the vector map(grid_frame) and the sensor(sensor_frame) .

#### Output topics
`/grid_map_wayarea` (grid_map::GridMap) is the resulting GridMap with values ranging from 0-255.
`/occupancy_wayarea` (nav_msgs::OccupancyGrid) is the resulting OccupancyGrid with values ranging from 0-255.

##### How to launch
It can be launched as follows:
 1. Using the Runtime Manager by clicking the `wayarea2grid` checkbox under the *Semantics* section in the Computing tab.
 2. From a sourced terminal by executing: `roslaunch object_map wayarea2grid.launch`

##### Parameters available in roslaunch and rosrun
* `sensor_frame` defines the coordinate frame of the vehicle origin (default value: velodyne).
* `grid_frame` defines the coordinate frame of the map (default: map).
* `grid_resolution` defines the equivalent value of a cell in the grid in meters. Smaller values result in better accuracy at the expense of memory and computing cost (default: 0.2).
* `grid_length_x` represents the width in meters of the OccupancyGrid around the origin of the PointCloud (default: 25).
* `grid_length_y` represents the height in meters of the OccupancyGrid around the origin of the PointCloud  (default: 0).
* `grid_position_x` indicates if the center of the OccupancyGrid will be shifted by this distance, left(-) or right(+) (default: 0).
* `grid_position_y` indicates if the center of the OccupancyGrid will be shifted by this distance, back(-) or front(+) (default: 0).

---

### grid_map_filter
This node can combine sensor, map, and perception data as well as other OccupancyGrids. It generates OccupancyGrids with useful data for navigation purposes. It publishes three different layers. More details are provided in the output topics section below.

#### Input topics
`/vector_map` (vector_map_msgs::WayArea) from the VectorMap publisher.
`/tf` to obtain the transform between the vector map (grid_frame) and the sensor (sensor_frame).
`/realtime_cost_map` (nav_msgs::OccupancyGrid) calculated by `points2costmap` in this package.

#### Output topics
`/filtered_grid_map` (grid_map::GridMap) which contains 3 layers and is also published as in regular OccupancyGrid messages.
`distance_transform` (nav_msgs::OccupancyGrid) applies a distance transform to the OccupancyGrid obtained from the `/realtime_cost_map`, allowing us to obtain a gradient of probabilities surrounding the obstacles PointCloud.
`dist_wayarea` (nav_msgs::OccupancyGrid) contains a combination of the distance transform and the road data, as described in the `wayarea2grid` node.
`circle` (nav_msgs::OccupancyGrid) draws a circle surrounding the `/realtime_cost_map`.

The output topics are configured as described in the `grid_map` package, and the configuration file is inside the `config` folder of the package.

##### How to launch
It can be launched as follows:
 1. Using the Runtime Manager by clicking the `grid_map_filter` checkbox under the *Semantics* section in the Computing tab.
 2. From a sourced terminal by executing: `roslaunch object_map grid_map_filter.launch`.

##### Parameters available in roslaunch and rosrun
 * `map_frame` defines the coordinate system of the realtime costmap (default value: map).
 * `map_topic` defines the topic where the realtime costmap is being published (default: /realtime_cost_map).
 * `dist_transform_distance` defines the maximum distance to calculate the distance transform, in meters (default: 2.0).
 * `use_wayarea` indicates whether or not to use the road regions to filter the cost map (default: true).
 * `use_fill_circle` enables or disables the generation of the circle layer (default: true).
 * `fill_circle_cost_threshold` indicates the minimum cost value threshold value to decide if a circle will be drawn (default: 20)
 * `circle_radius` defines the radius of the circle, in meters (default: 1.7).

---

## Instruction Videos

### grid_map_filter

[![](https://img.youtube.com/vi/bl-nG8Zv-C0/0.jpg)](https://www.youtube.com/watch?v=bl-nG8Zv-C0)

### wayarea2grid

[![](https://img.youtube.com/vi/UkcO4V-0NOw/0.jpg)](https://www.youtube.com/watch?v=UkcO4V-0NOw)
