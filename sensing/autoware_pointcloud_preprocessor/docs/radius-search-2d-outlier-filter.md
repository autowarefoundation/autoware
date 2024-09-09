# radius_search_2d_outlier_filter

## Purpose

The purpose is to remove point cloud noise such as insects and rain.

## Inner-workings / Algorithms

> RadiusOutlierRemoval filter which removes all indices in its input cloud that don’t have at least some number of neighbors within a certain range.

The description above is quoted from [1]. `pcl::search::KdTree` [2] is used to implement this package.

![radius_search_2d_outlier_filter_picture](./image/outlier_filter-radius_search_2d.drawio.svg)

## Inputs / Outputs

This implementation inherits `autoware::pointcloud_preprocessor::Filter` class, please refer [README](../README.md).

## Parameters

### Node Parameters

This implementation inherits `autoware::pointcloud_preprocessor::Filter` class, please refer [README](../README.md).

### Core Parameters

{{ json_to_markdown("sensing/autoware_pointcloud_preprocessor/schema/radius_search_2d_outlier_filter_node.schema.json") }}

## Assumptions / Known limits

Since the method is to count the number of points contained in the cylinder with the direction of gravity as the direction of the cylinder axis, it is a prerequisite that the ground has been removed.

## (Optional) Error detection and handling

## (Optional) Performance characterization

## References/External links

[1] <https://pcl.readthedocs.io/projects/tutorials/en/latest/remove_outliers.html>

[2] <https://pcl.readthedocs.io/projects/tutorials/en/latest/kdtree_search.html#kdtree-search>

## (Optional) Future extensions / Unimplemented parts
