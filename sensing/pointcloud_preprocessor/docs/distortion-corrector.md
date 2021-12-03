# distortion_corrector

## Purpose

The `distortion_corrector` is a node that compensates pointcloud distortion caused by ego vehicle's movement during 1 scan.

## Inner-workings / Algorithms

## Inputs / Outputs

### Input

| Name                      | Type                                              | Description      |
| ------------------------- | ------------------------------------------------- | ---------------- |
| `~/input/points`          | `sensor_msgs::msg::PointCloud2`                   | reference points |
| `~/input/velocity_report` | `autoware_auto_vehicle_msgs::msg::VelocityReport` | vehicle velocity |

### Output

| Name              | Type                            | Description     |
| ----------------- | ------------------------------- | --------------- |
| `~/output/points` | `sensor_msgs::msg::PointCloud2` | filtered points |

## Parameters

### Core Parameters

| Name                   | Type   | Default Value | Description           |
| ---------------------- | ------ | ------------- | --------------------- |
| `timestamp_field_name` | string | "time_stamp"  | time stamp field name |

## Assumptions / Known limits

## (Optional) Error detection and handling

## (Optional) Performance characterization

## (Optional) References/External links

## (Optional) Future extensions / Unimplemented parts
