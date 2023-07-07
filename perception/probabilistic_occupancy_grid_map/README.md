# probabilistic_occupancy_grid_map

## Purpose

This package outputs the probability of having an obstacle as occupancy grid map.
![pointcloud_based_occupancy_grid_map_sample_image](./image/pointcloud_based_occupancy_grid_map_sample_image.gif)

## References/External links

- [Pointcloud based occupancy grid map](pointcloud-based-occupancy-grid-map.md)
- [Laserscan based occupancy grid map](laserscan-based-occupancy-grid-map.md)

## Settings

Occupancy grid map is generated on `map_frame`, and grid orientation is fixed.

You may need to choose `scan_origin_frame` and `gridmap_origin_frame` which means sensor origin and gridmap origin respectively. Especially, set your main LiDAR sensor frame (e.g. `velodyne_top` in sample_vehicle) as a `scan_origin_frame` would result in better performance.

![image_for_frame_parameter_visualization](./image/gridmap_frame_settings.drawio.svg)

### Each config parameters

Config parameters are managed in `config/*.yaml` and here shows its outline.

- Pointcloud based occupancy grid map

| Ros param name                               | Default value |
| -------------------------------------------- | ------------- |
| map_frame                                    | "map"         |
| base_link_frame                              | "base_link"   |
| scan_origin_frame                            | "base_link"   |
| gridmap_origin_frame                         | "base_link"   |
| use_height_filter                            | true          |
| enable_single_frame_mode                     | false         |
| filter_obstacle_pointcloud_by_raw_pointcloud | false         |
| map_length                                   | 150.0 [m]     |
| map_resolution                               | 0.5 [m]       |
| use_projection                               | false         |
| projection_dz_threshold                      | 0.01          |
| obstacle_separation_threshold                | 1.0           |
| input_obstacle_pointcloud                    | true          |
| input_obstacle_and_raw_pointcloud            | true          |

- Laserscan based occupancy grid map

| Ros param name           | Default value |
| ------------------------ | ------------- |
| map_length               | 150 [m]       |
| map_width                | 150 [m]       |
| map_resolution           | 0.5 [m]       |
| use_height_filter        | true          |
| enable_single_frame_mode | false         |
| map_frame                | "map"         |
| base_link_frame          | "base_link"   |
| scan_origin_frame        | "base_link"   |
| gridmap_origin_frame     | "base_link"   |

## Other parameters

Additional argument is shown below:

| Name                                | Default                        | Description                                                                                   |
| ----------------------------------- | ------------------------------ | --------------------------------------------------------------------------------------------- |
| `use_multithread`                   | `false`                        | whether to use multithread                                                                    |
| `use_intra_process`                 | `false`                        |                                                                                               |
| `map_origin`                        | ``                             | parameter to override `map_origin_frame` which means grid map origin                          |
| `scan_origin`                       | ``                             | parameter to override `scan_origin_frame` which means scanning center                         |
| `output`                            | `occupancy_grid`               | output name                                                                                   |
| `use_pointcloud_container`          | `false`                        |                                                                                               |
| `container_name`                    | `occupancy_grid_map_container` |                                                                                               |
| `input_obstacle_pointcloud`         | `false`                        | only for laserscan based method. If true, the node subscribe obstacle pointcloud              |
| `input_obstacle_and_raw_pointcloud` | `true`                         | only for laserscan based method. If true, the node subscribe both obstacle and raw pointcloud |
