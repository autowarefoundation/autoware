# Scan Ground Filter

## Purpose

The purpose of this node is that remove the ground points from the input pointcloud.

## Inner-workings / Algorithms

This algorithm works by following steps,

1. Divide whole pointclouds into groups by horizontal angle and sort by xy-distance.
2. Check the distance and vertical angle of the point one by one.
3. Set a center of the ground contact point of the rear or front wheels as the initial point.
4. Check vertical angle between the points. If the angle from the initial point is larger than "global_slope_max", the point is classified as "no ground".
5. If the angle from the previous point is larger than "local_max_slope", the point is classified as "no ground".
6. Otherwise the point is labeled as "ground point".
7. If the distance from the last checked point is close, ignore any vertical angle and set current point attribute to the same as the last point.

## Inputs / Outputs

This implementation inherits `pointcloud_preprocessor::Filter` class, please refer [README](../README.md).

## Parameters

### Node Parameters

This implementation inherits `pointcloud_preprocessor::Filter` class, please refer [README](../README.md).

#### Core Parameters

| Name                              | Type   | Default Value | Description                                                                   |
| --------------------------------- | ------ | ------------- | ----------------------------------------------------------------------------- |
| `base_frame`                      | string | "base_link"   | base_link frame                                                               |
| `global_slope_max`                | double | 8.0           | The global angle to classify as the ground or object [deg]                    |
| `local_max_slope`                 | double | 6.0           | The local angle to classify as the ground or object [deg]                     |
| `radial_divider_angle`            | double | 1.0           | The angle which divide the whole pointcloud to sliced group [deg]             |
| `split_points_distance_tolerance` | double | 0.2           | The xy-distance threshold to to distinguishing far and near [m]               |
| `split_height_distance`           | double | 0.2           | The height threshold to distinguishing far and near [m]                       |
| `use_virtual_ground_point`        | bool   | true          | whether to use the ground center of front wheels as the virtual ground point. |

## Assumptions / Known limits

## (Optional) Error detection and handling

## (Optional) Performance characterization

## (Optional) References/External links

## (Optional) Future extensions / Unimplemented parts

- Horizontal check for classification is not implemented yet.
- Output ground visibility for diagnostic is not implemented yet.
