# voxel_grid_outlier_filter

## Purpose

The purpose is to remove point cloud noise such as insects and rain.

## Inner-workings / Algorithms

Removing point cloud noise based on the number of points existing within a voxel.
The [radius_search_2d_outlier_filter](./radius-search-2d-outlier-filter.md) is better for accuracy, but this method has the advantage of low calculation cost.

![voxel_grid_outlier_filter_picture](./image/outlier_filter-voxel_grid.drawio.svg)

## Inputs / Outputs

This implementation inherits `autoware::pointcloud_preprocessor::Filter` class, please refer [README](../README.md).

## Parameters

### Node Parameters

This implementation inherits `autoware::pointcloud_preprocessor::Filter` class, please refer [README](../README.md).

### Core Parameters

{{ json_to_markdown("sensing/autoware_pointcloud_preprocessor/schema/voxel_grid_outlier_filter_node.schema.json") }}

## Assumptions / Known limits

## (Optional) Error detection and handling

## (Optional) Performance characterization

## (Optional) References/External links

## (Optional) Future extensions / Unimplemented parts
