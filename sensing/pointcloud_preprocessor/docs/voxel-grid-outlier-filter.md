# voxel_grid_outlier_filter

## Purpose

The purpose is to remove point cloud noise such as insects and rain.

## Inner-workings / Algorithms

Removing point cloud noise based on the number of points existing within a voxel.
The [radius_search_2d_outlier_filter](./radius-search-2d-outlier-filter.md) is better for accuracy, but this method has the advantage of low calculation cost.

![voxel_grid_outlier_filter_picture](./image/outlier_filter-voxel_grid.drawio.svg)

## Inputs / Outputs

This implementation inherits `pointcloud_preprocessor::Filter` class, please refer [README](../README.md).

## Parameters

### Node Parameters

This implementation inherits `pointcloud_preprocessor::Filter` class, please refer [README](../README.md).

### Core Parameters

| Name                     | Type   | Default Value | Description                                |
| ------------------------ | ------ | ------------- | ------------------------------------------ |
| `voxel_size_x`           | double | 0.3           | the voxel size along x-axis [m]            |
| `voxel_size_y`           | double | 0.3           | the voxel size along y-axis [m]            |
| `voxel_size_z`           | double | 0.1           | the voxel size along z-axis [m]            |
| `voxel_points_threshold` | int    | 2             | the minimum number of points in each voxel |

## Assumptions / Known limits

## (Optional) Error detection and handling

## (Optional) Performance characterization

## (Optional) References/External links

## (Optional) Future extensions / Unimplemented parts
