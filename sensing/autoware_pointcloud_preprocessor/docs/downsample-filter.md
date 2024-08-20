# downsample_filter

## Purpose

The `downsample_filter` is a node that reduces the number of points.

## Inner-workings / Algorithms

### Approximate Downsample Filter

`pcl::VoxelGridNearestCentroid` is used. The algorithm is described in [autoware_pcl_extensions](../../autoware_pcl_extensions/README.md)

### Random Downsample Filter

`pcl::RandomSample` is used, which points are sampled with uniform probability.

### Voxel Grid Downsample Filter

`pcl::VoxelGrid` is used, which points in each voxel are approximated with their centroid.

### Pickup Based Voxel Grid Downsample Filter

This algorithm samples a single actual point existing within the voxel, not the centroid. The computation cost is low compared to Centroid Based Voxel Grid Filter.

## Inputs / Outputs

These implementations inherit `autoware::pointcloud_preprocessor::Filter` class, please refer [README](../README.md).

## Parameters

### Note Parameters

These implementations inherit `autoware::pointcloud_preprocessor::Filter` class, please refer [README](../README.md).

### Core Parameters

#### Approximate Downsample Filter

| Name           | Type   | Default Value | Description      |
| -------------- | ------ | ------------- | ---------------- |
| `voxel_size_x` | double | 0.3           | voxel size x [m] |
| `voxel_size_y` | double | 0.3           | voxel size y [m] |
| `voxel_size_z` | double | 0.1           | voxel size z [m] |

### Random Downsample Filter

| Name         | Type | Default Value | Description                     |
| ------------ | ---- | ------------- | ------------------------------- |
| `sample_num` | int  | 1500          | number of indices to be sampled |

### Voxel Grid Downsample Filter

{{ json_to_markdown("sensing/autoware_pointcloud_preprocessor/schema/voxel_grid_downsample_filter_node.schema.json") }}

### Pickup Based Voxel Grid Downsample Filter

| Name           | Type   | Default Value | Description      |
| -------------- | ------ | ------------- | ---------------- |
| `voxel_size_x` | double | 1.0           | voxel size x [m] |
| `voxel_size_y` | double | 1.0           | voxel size y [m] |
| `voxel_size_z` | double | 1.0           | voxel size z [m] |

## Assumptions / Known limits

<!-- cspell: ignore martinus -->

This implementation uses the `robin_hood.h` hashing library by martinus, available under the MIT License at [martinus/robin-hood-hashing](https://github.com/martinus/robin-hood-hashing) on GitHub. Special thanks to martinus for this contribution.

## (Optional) Error detection and handling

## (Optional) Performance characterization

## (Optional) References/External links

## (Optional) Future extensions / Unimplemented parts
