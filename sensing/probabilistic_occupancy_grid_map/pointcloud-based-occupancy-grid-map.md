# pointcloud based occupancy grid map

![pointcloud_based_occupancy_grid_map_sample_image](./image/pointcloud_based_occupancy_grid_map_sample_image.gif)

## Inner-workings / Algorithms

### 1st step

First of all, obstacle and raw pointcloud as input are transformed into a polar coordinate system and divided into bin per angle_increment.
At this time, each point belonging to each bin is stored as range data. In addition, the x,y information in the map coordinate is also stored for ray trace on map coordinate.
The bin contains the following information for each point

- range data from origin of raytrace
- x on map coordinate
- y on map coordinate

![pointcloud_based_occupancy_grid_map_bev](./image/pointcloud_based_occupancy_grid_map_bev.svg)

The following figure shows each of the bins from side view.
![pointcloud_based_occupancy_grid_map_side_view](./image/pointcloud_based_occupancy_grid_map_side_view.svg)

### 2nd step

The ray trace is performed in three steps for each cell.
The ray trace is done by Bresenham's line algorithm.
![Bresenham's line algorithm](./image/bresenham.svg)

1. Initialize freespace to the farthest point of each bin.

   ![pointcloud_based_occupancy_grid_map_side_view_1st](./image/pointcloud_based_occupancy_grid_map_side_view_1st.svg)

2. Fill in the unknown cells.
   Assume that unknown is behind the obstacle, since the back of the obstacle is a blind spot.
   Therefore, the unknown are assumed to be the cells that are more than a distance margin from each obstacle point.

   ![pointcloud_based_occupancy_grid_map_side_view_2nd](./image/pointcloud_based_occupancy_grid_map_side_view_2nd.svg)

   There are three reasons for setting a distance margin.

   - It is unlikely that a point on the ground will be immediately behind an obstacle.
   - The obstacle point cloud is processed and may not match the raw pointcloud.
   - The input may be inaccurate and obstacle points may not be determined as obstacles.

3. Fill in the occupied cells.
   Fill in the point where the obstacle point is located with occupied.
   In addition, If the distance between obstacle points is less than or equal to the distance margin, it is filled with occupied because the input may be inaccurate and obstacle points may not be determined as obstacles.

   ![pointcloud_based_occupancy_grid_map_side_view_3rd](./image/pointcloud_based_occupancy_grid_map_side_view_3rd.svg)

### 3rd step

Using the previous occupancy grid map, update the existence probability using a binary Bayesian filter (1). Also, the unobserved cells are time-decayed like the system noise of the Kalman filter (2).

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
| `~/input/obstacle_pointcloud` | `sensor_msgs::PointCloud2` | obstacle pointcloud                                            |
| `~/input/raw_pointcloud`      | `sensor_msgs::PointCloud2` | The overall point cloud used to input the obstacle point cloud |

### Output

| Name                          | Type                      | Description        |
| ----------------------------- | ------------------------- | ------------------ |
| `~/output/occupancy_grid_map` | `nav_msgs::OccupancyGrid` | occupancy grid map |

## Parameters

### Node Parameters

| Name                | Type   | Description                                                                                                                      |
| ------------------- | ------ | -------------------------------------------------------------------------------------------------------------------------------- |
| `map_frame`         | string | map frame                                                                                                                        |
| `base_link_frame`   | string | base_link frame                                                                                                                  |
| `use_height_filter` | bool   | whether to height filter for `~/input/obstacle_pointcloud` and `~/input/raw_pointcloud`? By default, the height is set to -1~2m. |
| `map_length`        | double | The length of the map. -100 if it is 50~50[m]                                                                                    |
| `map_resolution`    | double | The map cell resolution [m]                                                                                                      |

## Assumptions / Known limits

In several places we have modified the external code written in BSD3 license.

- occupancy_grid_map.hpp
- cost_value.hpp
- occupancy_grid_map.cpp

## (Optional) Error detection and handling

## (Optional) Performance characterization

## (Optional) References/External links

## (Optional) Future extensions / Unimplemented parts

- The update probability of the binary Bayesian filter is currently hard-coded and requires a code change to be modified.
- Since there is no special support for moving objects, the probability of existence is not increased for fast objects.
