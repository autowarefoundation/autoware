# vector_map_filter

## Purpose

The `vector_map_filter` is a node that removes points on the outside of lane by using vector map.

## Inner-workings / Algorithms

## Inputs / Outputs

### Input

| Name                 | Type                                         | Description      |
| -------------------- | -------------------------------------------- | ---------------- |
| `~/input/points`     | `sensor_msgs::msg::PointCloud2`              | reference points |
| `~/input/vector_map` | `autoware_auto_mapping_msgs::msg::HADMapBin` | vector map       |

### Output

| Name              | Type                            | Description     |
| ----------------- | ------------------------------- | --------------- |
| `~/output/points` | `sensor_msgs::msg::PointCloud2` | filtered points |

## Parameters

### Core Parameters

| Name           | Type   | Default Value | Description |
| -------------- | ------ | ------------- | ----------- |
| `voxel_size_x` | double | 0.04          | voxel size  |
| `voxel_size_y` | double | 0.04          | voxel size  |

## Assumptions / Known limits

## (Optional) Error detection and handling

## (Optional) Performance characterization

## (Optional) References/External links

## (Optional) Future extensions / Unimplemented parts
