# pointcloud_accumulator

## Purpose

The `pointcloud_accumulator` is a node that accumulates pointclouds for a given amount of time.

## Inner-workings / Algorithms

## Inputs / Outputs

### Input

| Name             | Type                            | Description      |
| ---------------- | ------------------------------- | ---------------- |
| `~/input/points` | `sensor_msgs::msg::PointCloud2` | reference points |

### Output

| Name              | Type                            | Description     |
| ----------------- | ------------------------------- | --------------- |
| `~/output/points` | `sensor_msgs::msg::PointCloud2` | filtered points |

## Parameters

### Core Parameters

{{ json_to_markdown("sensing/autoware_pointcloud_preprocessor/schema/pointcloud_accumulator_node.schema.json") }}

## Assumptions / Known limits

## (Optional) Error detection and handling

## (Optional) Performance characterization

## (Optional) References/External links

## (Optional) Future extensions / Unimplemented parts
