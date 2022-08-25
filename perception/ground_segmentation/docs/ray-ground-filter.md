# Ray Ground Filter

## Purpose

The purpose of this node is that remove the ground points from the input pointcloud.

## Inner-workings / Algorithms

The points is separated radially (Ray), and the ground is classified for each Ray sequentially from the point close to ego-vehicle based on the geometric information such as the distance and angle between the points.

![ray-xy](./image/ground_filter-ray-xy.drawio.svg)

## Inputs / Outputs

This implementation inherits `pointcloud_preprocessor::Filter` class, please refer [README](../README.md).

## Parameters

### Node Parameters

This implementation inherits `pointcloud_preprocessor::Filter` class, please refer [README](../README.md).

### Core Parameters

![ray-xz](./image/ground_filter-ray-xz.drawio.svg)

| Name                          | Type   | Description                                                                                                                                                                                                                    |
| ----------------------------- | ------ | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------ |
| `input_frame`                 | string | frame id of input pointcloud                                                                                                                                                                                                   |
| `output_frame`                | string | frame id of output pointcloud                                                                                                                                                                                                  |
| `general_max_slope`           | double | The triangle created by `general_max_slope` is called the global cone. If the point is outside the global cone, it is judged to be a point that is no on the ground                                                            |
| `initial_max_slope`           | double | Generally, the point where the object first hits is far from ego-vehicle because of sensor blind spot, so resolution is different from that point and thereafter, so this parameter exists to set a separate `local_max_slope` |
| `local_max_slope`             | double | The triangle created by `local_max_slope` is called the local cone. This parameter for classification based on the continuity of points                                                                                        |
| `min_height_threshold`        | double | This parameter is used instead of `height_threshold` because it's difficult to determine continuity in the local cone when the points are too close to each other.                                                             |
| `radial_divider_angle`        | double | The angle of ray                                                                                                                                                                                                               |
| `concentric_divider_distance` | double | Only check points which radius is larger than `concentric_divider_distance`                                                                                                                                                    |
| `reclass_distance_threshold`  | double | To check if point is to far from previous one, if so classify again                                                                                                                                                            |
| `min_x`                       | double | The parameter to set vehicle footprint manually                                                                                                                                                                                |
| `max_x`                       | double | The parameter to set vehicle footprint manually                                                                                                                                                                                |
| `min_y`                       | double | The parameter to set vehicle footprint manually                                                                                                                                                                                |
| `max_y`                       | double | The parameter to set vehicle footprint manually                                                                                                                                                                                |

## Assumptions / Known limits

The input_frame is set as parameter but it must be fixed as base_link for the current algorithm.

## (Optional) Error detection and handling

## (Optional) Performance characterization

## (Optional) References/External links

## (Optional) Future extensions / Unimplemented parts
