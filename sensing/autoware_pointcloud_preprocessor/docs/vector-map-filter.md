# vector_map_filter

## Purpose

The `vector_map_filter` is a node that removes points on the outside of lane by using vector map.

## Inner-workings / Algorithms

## Inputs / Outputs

### Input

| Name                 | Type                                    | Description      |
| -------------------- | --------------------------------------- | ---------------- |
| `~/input/points`     | `sensor_msgs::msg::PointCloud2`         | reference points |
| `~/input/vector_map` | `autoware_map_msgs::msg::LaneletMapBin` | vector map       |

### Output

| Name              | Type                            | Description     |
| ----------------- | ------------------------------- | --------------- |
| `~/output/points` | `sensor_msgs::msg::PointCloud2` | filtered points |

## Parameters

### Core Parameters

{{ json_to_markdown("sensing/autoware_pointcloud_preprocessor/schema/lanelet2_map_filter_node.schema.json") }}

## Assumptions / Known limits

## (Optional) Error detection and handling

## (Optional) Performance characterization

## (Optional) References/External links

## (Optional) Future extensions / Unimplemented parts
