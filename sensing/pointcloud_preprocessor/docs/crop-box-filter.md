# crop_box_filter

## Purpose

The `crop_box_filter` is a node that removes points with in a given box region. This filter is used to remove the points that hit the vehicle itself.

## Inner-workings / Algorithms

`pcl::CropBox` is used, which filters all points inside a given box.

## Inputs / Outputs

This implementation inherit `pointcloud_preprocessor::Filter` class, please refer [README](../README.md).

## Parameters

### Node Parameters

This implementation inherit `pointcloud_preprocessor::Filter` class, please refer [README](../README.md).

### Core Parameters

| Name    | Type   | Default Value | Description                               |
| ------- | ------ | ------------- | ----------------------------------------- |
| `min_x` | double | -1.0          | x-coordinate minimum value for crop range |
| `max_x` | double | 1.0           | x-coordinate maximum value for crop range |
| `min_y` | double | -1.0          | y-coordinate minimum value for crop range |
| `max_y` | double | 1.0           | y-coordinate maximum value for crop range |
| `min_z` | double | -1.0          | z-coordinate minimum value for crop range |
| `max_z` | double | 1.0           | z-coordinate maximum value for crop range |

## Assumptions / Known limits

## (Optional) Error detection and handling

## (Optional) Performance characterization

## (Optional) References/External links

## (Optional) Future extensions / Unimplemented parts
