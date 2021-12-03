# tier4_pcl_extensions

## Purpose

The `tier4_pcl_extensions` is a pcl extension library. The voxel grid filter in this package works with a different algorithm than the original one.

## Inner-workings / Algorithms

### Original Algorithm [1]

1. create a 3D voxel grid over the input pointcloud data
2. calculate centroid in each voxel
3. all the points are approximated with their centroid

### Extended Algorithm

1. create a 3D voxel grid over the input pointcloud data
2. calculate centroid in each voxel
3. **all the points are approximated with the closest point to their centroid**

## Inputs / Outputs

## Parameters

## Assumptions / Known limits

## (Optional) Error detection and handling

## (Optional) Performance characterization

## (Optional) References/External links

[1] <https://pointclouds.org/documentation/tutorials/voxel_grid.html>

## (Optional) Future extensions / Unimplemented parts
