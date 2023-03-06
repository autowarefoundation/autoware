# probabilistic_occupancy_grid_map

## Purpose

This package outputs the probability of having an obstacle as occupancy grid map.
![pointcloud_based_occupancy_grid_map_sample_image](./image/pointcloud_based_occupancy_grid_map_sample_image.gif)

## Settings

Occupancy grid map is generated on `map_frame`, and grid orientation is fixed.

You may need to choose `output_frame` which means grid map origin. Default is `base_link`, but your main LiDAR sensor frame (e.g. `velodyne_top` in sample_vehicle) would be the better choice.

### Each config paramters

Config parameters are managed in `config/*.yaml` and here shows its outline.

- Pointcloud based occupancy grid map

| Ros param name                    | Default value |
| --------------------------------- | ------------- |
| map_frame                         | "map"         |
| base_link_frame                   | "base_link"   |
| output_frame                      | "base_link"   |
| use_height_filter                 | true          |
| enable_single_frame_mode          | false         |
| map_length                        | 100.0 [m]     |
| map_width                         | 100.0 [m]     |
| map_resolution                    | 0.5 [m]       |
| input_obstacle_pointcloud         | true          |
| input_obstacle_and_raw_pointcloud | true          |

- Laserscan based occupancy grid map

| Ros param name           | Default value |
| ------------------------ | ------------- |
| map_length               | 100 [m]       |
| map_resolution           | 0.5 [m]       |
| use_height_filter        | true          |
| enable_single_frame_mode | false         |
| map_frame                | "map"         |
| base_link_frame          | "base_link"   |
| output_frame             | "base_link"   |

## References/External links

- [Pointcloud based occupancy grid map](pointcloud-based-occupancy-grid-map.md)
- [Laserscan based occupancy grid map](laserscan-based-occupancy-grid-map.md)
