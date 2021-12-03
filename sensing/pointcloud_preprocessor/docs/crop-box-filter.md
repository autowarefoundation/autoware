# box_crop_filter

## Purpose

The `box_crop_filter` is a node that removes points with in a given box region. This filter is used to remove the points that hit the vehicle itself.

## Inner-workings / Algorithms

## Inputs / Outputs

| Name             | Type                            | Description      |
| ---------------- | ------------------------------- | ---------------- |
| `~/input/points` | `sensor_msgs::msg::PointCloud2` | reference points |

### Output

| Name              | Type                            | Description     |
| ----------------- | ------------------------------- | --------------- |
| `~/output/points` | `sensor_msgs::msg::PointCloud2` | filtered points |

## Parameters

### Core Parameters

| Name    | Type   | Default Value | Description                               |
| ------- | ------ | ------------- | ----------------------------------------- |
| `min_x` | double | -1.0          | x-coordinate minimum value for crop range |
| `max_x` | double | 1.0           | x-coordinate maximum value for crop range |
| `min_y` | double | -1.0          | y-coordinate minimum value for crop range |
| `max_y` | double | 1.0           | y-coordinate maximum value for crop range |
| `min_z` | double | -1.0          | z-coordinate minimum value for crop range |
| `max_z` | double | 1.0           | z-coordinate maximum value for crop range |

## Assumptions / Known limits

## (Optional) Error detection and handling

## (Optional) Performance characterization

## (Optional) References/External links

## (Optional) Future extensions / Unimplemented parts
