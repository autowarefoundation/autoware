# passthrough_filter

## Purpose

The `passthrough_filter` is a node that removes points on the outside of a range in a given field (e.g. x, y, z, intensity, ring, etc).

## Inner-workings / Algorithms

## Inputs / Outputs

### Input

| Name              | Type                            | Description       |
| ----------------- | ------------------------------- | ----------------- |
| `~/input/points`  | `sensor_msgs::msg::PointCloud2` | reference points  |
| `~/input/indices` | `pcl_msgs::msg::Indices`        | reference indices |

### Output

| Name              | Type                            | Description     |
| ----------------- | ------------------------------- | --------------- |
| `~/output/points` | `sensor_msgs::msg::PointCloud2` | filtered points |

## Parameters

### Core Parameters

| Name                    | Type   | Default Value | Description                                            |
| ----------------------- | ------ | ------------- | ------------------------------------------------------ |
| `filter_limit_min`      | int    | 0             | minimum allowed field value                            |
| `filter_limit_max`      | int    | 127           | maximum allowed field value                            |
| `filter_field_name`     | string | "ring"        | filtering field name                                   |
| `keep_organized`        | bool   | false         | flag to keep indices structure                         |
| `filter_limit_negative` | bool   | false         | flag to return whether the data is inside limit or not |

## Assumptions / Known limits

## (Optional) Error detection and handling

## (Optional) Performance characterization

## (Optional) References/External links

## (Optional) Future extensions / Unimplemented parts
