# compare_map_segmentation

## Purpose

The `compare_map_segmentation` is a node that filters the ground points from the input pointcloud by using map info (e.g. pcd, elevation map).

## Inner-workings / Algorithms

### Compare Elevation Map Filter

Compare the z of the input points with the value of elevation_map. The height difference is calculated by the binary integration of neighboring cells. Remove points whose height difference is below the `height_diff_thresh`.

<p align="center">
  <img src="./media/compare_elevation_map.png" width="1000">
</p>

### Distance Based Compare Map Filter

WIP

### Voxel Based Approximate Compare Map Filter

WIP

### Voxel Based Compare Map Filter

WIP

### Voxel Distance based Compare Map Filter

WIP

## Inputs / Outputs

### Compare Elevation Map Filter

#### Input

| Name                    | Type                            | Description      |
| ----------------------- | ------------------------------- | ---------------- |
| `~/input/points`        | `sensor_msgs::msg::PointCloud2` | reference points |
| `~/input/elevation_map` | `grid_map::msg::GridMap`        | elevation map    |

#### Output

| Name              | Type                            | Description     |
| ----------------- | ------------------------------- | --------------- |
| `~/output/points` | `sensor_msgs::msg::PointCloud2` | filtered points |

### Other Filters

#### Input

| Name             | Type                            | Description      |
| ---------------- | ------------------------------- | ---------------- |
| `~/input/points` | `sensor_msgs::msg::PointCloud2` | reference points |
| `~/input/map`    | `grid_map::msg::GridMap`        | map              |

#### Output

| Name              | Type                            | Description     |
| ----------------- | ------------------------------- | --------------- |
| `~/output/points` | `sensor_msgs::msg::PointCloud2` | filtered points |

## Parameters

### Core Parameters

| Name                 | Type   | Description                                                                     | Default value |
| :------------------- | :----- | :------------------------------------------------------------------------------ | :------------ |
| `map_layer_name`     | string | elevation map layer name                                                        | elevation     |
| `map_frame`          | float  | frame_id of the map that is temporarily used before elevation_map is subscribed | map           |
| `height_diff_thresh` | float  | Remove points whose height difference is below this value [m]                   | 0.15          |

## Assumptions / Known limits

## (Optional) Error detection and handling

## (Optional) Performance characterization

## (Optional) References/External links

## (Optional) Future extensions / Unimplemented parts
