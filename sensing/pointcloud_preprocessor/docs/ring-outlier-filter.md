# ring_outlier_filter

## Purpose

The purpose is to remove point cloud noise such as insects and rain.

## Inner-workings / Algorithms

A method of operating scan in chronological order and removing noise based on the rate of change in the distance between points

![ring_outlier_filter](./image/outlier_filter-ring.drawio.svg)

## Inputs / Outputs

This implementation inherits `pointcloud_preprocessor::Filter` class, please refer [README](../README.md).

## Parameters

### Node Parameters

This implementation inherits `pointcloud_preprocessor::Filter` class, please refer [README](../README.md).

### Core Parameters

| Name                      | Type   | Default Value | Description |
| ------------------------- | ------ | ------------- | ----------- |
| `distance_ratio`          | double | 1.03          |             |
| `object_length_threshold` | double | 0.1           |             |
| `num_points_threshold`    | int    | 4             |             |

## Assumptions / Known limits

It is a prerequisite to input a scan point cloud in chronological order. In this repository it is defined as blow structure (please refer to [PointXYZIRADT](https://github.com/tier4/AutowareArchitectureProposal.iv/blob/5d8dff0db51634f0c42d2a3e87ca423fbee84348/sensing/preprocessor/pointcloud/pointcloud_preprocessor/include/pointcloud_preprocessor/outlier_filter/ring_outlier_filter_nodelet.hpp#L53-L62)).

- X: x
- Y: y
- z: z
- I: intensity
- R: ring
- A :azimuth
- D: distance
- T: time_stamp

## (Optional) Error detection and handling

## (Optional) Performance characterization

## (Optional) References/External links

## (Optional) Future extensions / Unimplemented parts
