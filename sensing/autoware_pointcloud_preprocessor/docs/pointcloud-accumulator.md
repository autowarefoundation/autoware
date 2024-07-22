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

| Name                     | Type   | Default Value | Description             |
| ------------------------ | ------ | ------------- | ----------------------- |
| `accumulation_time_sec`  | double | 2.0           | accumulation period [s] |
| `pointcloud_buffer_size` | int    | 50            | buffer size             |

## Assumptions / Known limits

## (Optional) Error detection and handling

## (Optional) Performance characterization

## (Optional) References/External links

## (Optional) Future extensions / Unimplemented parts
