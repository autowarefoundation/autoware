# vector_map_inside_area_filter

## Purpose

The `vector_map_inside_area_filter` is a node that removes points inside the vector map area that has given type by parameter.

## Inner-workings / Algorithms

- Get the vector map area that has given type by parameter of `polygon_type`
- Extract the vector map area that intersects with the bounding box of input points to reduce the calculation cost
- Create the 2D polygon from the extracted vector map area
- Remove input points inside the polygon

![vector_map_inside_area_filter_figure](./image/vector_map_inside_area_filter_overview.svg)

## Inputs / Outputs

This implementation inherits `pointcloud_preprocessor::Filter` class, so please see also [README](../README.md).

### Input

| Name                 | Type                                         | Description                          |
| -------------------- | -------------------------------------------- | ------------------------------------ |
| `~/input`            | `sensor_msgs::msg::PointCloud2`              | input points                         |
| `~/input/vector_map` | `autoware_auto_mapping_msgs::msg::HADMapBin` | vector map used for filtering points |

### Output

| Name       | Type                            | Description     |
| ---------- | ------------------------------- | --------------- |
| `~/output` | `sensor_msgs::msg::PointCloud2` | filtered points |

### Core Parameters

| Name           | Type   | Description                 |
| -------------- | ------ | --------------------------- |
| `polygon_type` | string | polygon type to be filtered |

## Assumptions / Known limits
