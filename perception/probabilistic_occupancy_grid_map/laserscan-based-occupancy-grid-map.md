# laserscan based occupancy grid map

![laserscan_based_occupancy_grid_map_sample_image](./image/laserscan_based_occupancy_grid_map_sample_image.png)

## Inner-workings / Algorithms

The basic idea is to take a 2D laserscan and ray trace it to create a time-series processed occupancy grid map.

1. the node take a laserscan and make an occupancy grid map with one frame. ray trace is done by Bresenham's line algorithm.
   ![Bresenham's line algorithm](./image/bresenham.svg)
2. Optionally, obstacle point clouds and raw point clouds can be received and reflected in the occupancy grid map. The reason is that laserscan only uses the most foreground point in the polar coordinate system, so it throws away a lot of information. As a result, the occupancy grid map is almost an UNKNOWN cell.
   Therefore, the obstacle point cloud and the raw point cloud are used to reflect what is judged to be the ground and what is judged to be an obstacle in the occupancy grid map.
   ![Bresenham's line algorithm](./image/update_with_pointcloud.svg)
   The black and red dots represent raw point clouds, and the red dots represent obstacle point clouds. In other words, the black points are determined as the ground, and the red point cloud is the points determined as obstacles. The gray cells are represented as UNKNOWN cells.

3. Using the previous occupancy grid map, update the existence probability using a binary Bayesian filter (1). Also, the unobserved cells are time-decayed like the system noise of the Kalman filter (2).

```math
    \hat{P_{o}} = \frac{(P_{o} * P_{z})}{(P_{o} * P_{z} + (1 - P_{o}) * \bar{P_{z}})} \tag{1}
```

```math
    \hat{P_{o}} = \frac{(P_{o} + 0.5 * \frac{1}{ratio})}{(\frac{1}{ratio} + 1)} \tag{2}
```

## Inputs / Outputs

### Input

| Name                          | Type                       | Description                                                    |
| ----------------------------- | -------------------------- | -------------------------------------------------------------- |
| `~/input/laserscan`           | `sensor_msgs::LaserScan`   | laserscan                                                      |
| `~/input/obstacle_pointcloud` | `sensor_msgs::PointCloud2` | obstacle pointcloud                                            |
| `~/input/raw_pointcloud`      | `sensor_msgs::PointCloud2` | The overall point cloud used to input the obstacle point cloud |

### Output

| Name                          | Type                      | Description        |
| ----------------------------- | ------------------------- | ------------------ |
| `~/output/occupancy_grid_map` | `nav_msgs::OccupancyGrid` | occupancy grid map |

## Parameters

### Node Parameters

| Name                                | Type   | Description                                                                                                                                                    |
| ----------------------------------- | ------ | -------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `map_frame`                         | string | map frame                                                                                                                                                      |
| `base_link_frame`                   | string | base_link frame                                                                                                                                                |
| `input_obstacle_pointcloud`         | bool   | whether to use the optional obstacle point cloud? If this is true, `~/input/obstacle_pointcloud` topics will be received.                                      |
| `input_obstacle_and_raw_pointcloud` | bool   | whether to use the optional obstacle and raw point cloud? If this is true, `~/input/obstacle_pointcloud` and `~/input/raw_pointcloud` topics will be received. |
| `use_height_filter`                 | bool   | whether to height filter for `~/input/obstacle_pointcloud` and `~/input/raw_pointcloud`? By default, the height is set to -1~2m.                               |
| `map_length`                        | double | The length of the map. -100 if it is 50~50[m]                                                                                                                  |
| `map_resolution`                    | double | The map cell resolution [m]                                                                                                                                    |

## Assumptions / Known limits

In several places we have modified the external code written in BSD3 license.

- occupancy_grid_map.hpp
- cost_value.hpp
- occupancy_grid_map.cpp

## (Optional) Error detection and handling

## (Optional) Performance characterization

## (Optional) References/External links

Bresenham's_line_algorithm

- <https://en.wikipedia.org/wiki/Bresenham%27s_line_algorithm>
- <https://web.archive.org/web/20080528040104/http://www.research.ibm.com/journal/sj/041/ibmsjIVRIC.pdf>

## (Optional) Future extensions / Unimplemented parts

- The update probability of the binary Bayesian filter is currently hard-coded and requires a code change to be modified.
- Since there is no special support for moving objects, the probability of existence is not increased for fast objects.
