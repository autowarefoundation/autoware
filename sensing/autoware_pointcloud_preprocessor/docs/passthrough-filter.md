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

{{ json_to_markdown("sensing/autoware_pointcloud_preprocessor/schema/passthrough_filter_uint16_node.schema.json
") }}

## Assumptions / Known limits

## (Optional) Error detection and handling

## (Optional) Performance characterization

## (Optional) References/External links

## (Optional) Future extensions / Unimplemented parts
