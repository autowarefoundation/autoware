# concatenate_data

## Purpose

The `concatenate_data` is a node that concatenates multiple pointclouds acquired by multiple LiDARs into a pointcloud.

## Inner-workings / Algorithms

## Inputs / Outputs

### Input

| Name             | Type                                              | Description      |
| ---------------- | ------------------------------------------------- | ---------------- |
| `~/input/points` | `sensor_msgs::msg::Pointcloud2`                   | reference points |
| `~/input/twist`  | `autoware_auto_vehicle_msgs::msg::VelocityReport` | vehicle velocity |

### Output

| Name              | Type                            | Description     |
| ----------------- | ------------------------------- | --------------- |
| `~/output/points` | `sensor_msgs::msg::Pointcloud2` | filtered points |

## Parameters

| Name             | Type   | Default Value | Description                           |
| ---------------- | ------ | ------------- | ------------------------------------- |
| `input_frame`    | string | " "           | input frame id                        |
| `output_frame`   | string | " "           | output frame id                       |
| `max_queue_size` | int    | 5             | max queue size of input/output topics |

### Core Parameters

| Name          | Type   | Default Value | Description                                                                                                                                                                              |
| ------------- | ------ | ------------- | ---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `timeout_sec` | double | 0.1           | tolerance of time to publish next pointcloud [s]<br>When this time limit is exceeded, the filter concatenates and publishes pointcloud, even if not all the point clouds are subscribed. |

## Assumptions / Known limits

## (Optional) Error detection and handling

## (Optional) Performance characterization

## (Optional) References/External links

## (Optional) Future extensions / Unimplemented parts
