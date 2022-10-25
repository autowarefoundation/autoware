# Scan Ground Filter

## Purpose

The purpose of this node is that remove the ground points from the input pointcloud.

## Inner-workings / Algorithms

This algorithm works by following steps,

1. Divide whole pointclouds into groups by horizontal angle and sort by xy-distance.
2. Divide sorted pointclouds of each ray into grids
3. Check the xy distance to previous pointcloud, if the distance is large and previous pointcloud is "no ground" and the height level of current point greater than previous point, the current pointcloud is classified as no ground.
4. Check vertical angle of the point compared with previous ground grid
5. Check the height of the point compared with predicted ground level
6. If vertical angle is greater than local_slope_max and related height to predicted ground level is greater than "non ground height threshold", the point is classified as "non ground"
7. If the vertical angle is in range of [-local_slope_max, local_slope_max] or related height to predicted ground level is smaller than non_ground_height_threshold, the point is classified as "ground"
8. If the vertical angle is lower than -local_slope_max or the related height to ground level is greater than detection_range_z_max, the point will be classified as out of range

## Inputs / Outputs

This implementation inherits `pointcloud_preprocessor::Filter` class, please refer [README](../README.md).

## Parameters

### Node Parameters

This implementation inherits `pointcloud_preprocessor::Filter` class, please refer [README](../README.md).

#### Core Parameters

| Name                              | Type   | Default Value | Description                                                                                                                                       |
| --------------------------------- | ------ | ------------- | ------------------------------------------------------------------------------------------------------------------------------------------------- |
| `input_frame`                     | string | "base_link"   | frame id of input pointcloud                                                                                                                      |
| `output_frame`                    | string | "base_link"   | frame id of output pointcloud                                                                                                                     |
| `global_slope_max_angle_deg`      | double | 8.0           | The global angle to classify as the ground or object [deg]                                                                                        |
| `local_slope_max_angle_deg`       | double | 10.0          | The local angle to classify as the ground or object [deg]                                                                                         |
| `radial_divider_angle_deg`        | double | 1.0           | The angle which divide the whole pointcloud to sliced group [deg]                                                                                 |
| `split_points_distance_tolerance` | double | 0.2           | The xy-distance threshold to to distinguishing far and near [m]                                                                                   |
| `split_height_distance`           | double | 0.2           | The height threshold to distinguishing far and near [m]                                                                                           |
| `use_virtual_ground_point`        | bool   | true          | whether to use the ground center of front wheels as the virtual ground point.                                                                     |
| `detection_range_z_max`           | float  | 2.5           | Maximum height of detection range [m], applied only for elevation_grid_mode                                                                       |
| `center_pcl_shift`                | float  | 0.0           | The x-axis offset of addition LiDARs from vehicle center of mass [m], <br /> recommended to use only for additional LiDARs in elevation_grid_mode |
| `non_ground_height_threshold`     | float  | 0.2           | Height threshold of non ground objects [m], applied only for elevation_grid_mode                                                                  |
| `grid_mode_switch_radius`         | float  | 20.0          | The distance where grid division mode change from by distance to by vertical angle [m],<br /> applied only for elevation_grid_mode                |
| `grid_size_m`                     | float  | 0.5           | The first grid size [m], applied only for elevation_grid_mode                                                                                     |
| `gnd_grid_buffer_size`            | uint16 | 4             | Number of grids using to estimate local ground slope,<br /> applied only for elevation_grid_mode                                                  |
| `low_priority_region_x`           | float  | -20.0         | The non-zero x threshold in back side from which small objects detection is low priority [m]                                                      |
| `elevation_grid_mode`             | bool   | true          | Elevation grid scan mode option                                                                                                                   |

## Assumptions / Known limits

The input_frame is set as parameter but it must be fixed as base_link for the current algorithm.

## (Optional) Error detection and handling

## (Optional) Performance characterization

## (Optional) References/External links

<!-- cspell: ignore Shen Liang -->

The elevation grid idea is referred from "Shen Z, Liang H, Lin L, Wang Z, Huang W, Yu J. Fast Ground Segmentation for 3D LiDAR Point Cloud Based on Jump-Convolution-Process. Remote Sensing. 2021; 13(16):3239. <https://doi.org/10.3390/rs13163239>"

## (Optional) Future extensions / Unimplemented parts

- Horizontal check for classification is not implemented yet.
- Output ground visibility for diagnostic is not implemented yet.
