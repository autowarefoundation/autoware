# downsample_filter

## Purpose

The `downsample_filter` is a node that reduces the number of points.

## Inner-workings / Algorithms

### Approximate Downsample Filter

WIP

### Random Downsample Filter

WIP

### Voxel Grid Downsample Filter

WIP

## Inputs / Outputs

| Name             | Type                            | Description      |
| ---------------- | ------------------------------- | ---------------- |
| `~/input/points` | `sensor_msgs::msg::PointCloud2` | reference points |

### Output

| Name              | Type                            | Description     |
| ----------------- | ------------------------------- | --------------- |
| `~/output/points` | `sensor_msgs::msg::PointCloud2` | filtered points |

## Parameters

### Node Parameters

| Name           | Type   | Default Value | Description                     |
| -------------- | ------ | ------------- | ------------------------------- |
| `voxel_size_x` | double | 0.3           | voxel size x [m]                |
| `voxel_size_y` | double | 0.3           | voxel size y [m]                |
| `voxel_size_z` | double | 0.1           | voxel size z [m]                |
| `sample_num`   | int    | 1500          | number of indices to be sampled |

## Assumptions / Known limits

## (Optional) Error detection and handling

## (Optional) Performance characterization

## (Optional) References/External links

## (Optional) Future extensions / Unimplemented parts
