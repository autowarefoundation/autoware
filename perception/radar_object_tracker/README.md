# Radar Object Tracker

## Purpose

This package provides a radar object tracking node that processes sequences of detected objects to assign consistent identities to them and estimate their velocities.

## Inner-workings / Algorithms

This radar object tracker is a combination of data association and tracking algorithms.

<!-- In the future, you can add an overview image here -->
<!-- ![radar_object_tracker_overview](image/radar_object_tracker_overview.svg) -->

### Data Association

The data association algorithm matches detected objects to existing tracks.

### Tracker Models

The tracker models used in this package vary based on the class of the detected object.
See more details in the [models.md](models.md).

<!-- In the future, you can add flowcharts, state transitions, and other details about how this package works. -->

## Inputs / Outputs

### Input

| Name      | Type                                                  | Description      |
| --------- | ----------------------------------------------------- | ---------------- |
| `~/input` | `autoware_auto_perception_msgs::msg::DetectedObjects` | Detected objects |

### Output

| Name       | Type                                                 | Description     |
| ---------- | ---------------------------------------------------- | --------------- |
| `~/output` | `autoware_auto_perception_msgs::msg::TrackedObjects` | Tracked objects |

## Parameters

### Node Parameters

| Name                        | Type   | Default Value                 | Description                                                 |
| --------------------------- | ------ | ----------------------------- | ----------------------------------------------------------- |
| `publish_rate`              | double | 30.0                          | The rate at which to publish the output messages            |
| `world_frame_id`            | string | "world"                       | The frame ID of the world coordinate system                 |
| `enable_delay_compensation` | bool   | false                         | Whether to enable delay compensation                        |
| `tracking_config_directory` | string | ""                            | The directory containing the tracking configuration files   |
| `enable_logging`            | bool   | false                         | Whether to enable logging                                   |
| `logging_file_path`         | string | "~/.ros/association_log.json" | The path to the file where logs should be written           |
| `can_assign_matrix`         | array  |                               | An array of integers used in the data association algorithm |
| `max_dist_matrix`           | array  |                               | An array of doubles used in the data association algorithm  |
| `max_area_matrix`           | array  |                               | An array of doubles used in the data association algorithm  |
| `min_area_matrix`           | array  |                               | An array of doubles used in the data association algorithm  |
| `max_rad_matrix`            | array  |                               | An array of doubles used in the data association algorithm  |
| `min_iou_matrix`            | array  |                               | An array of doubles used in the data association algorithm  |

## Assumptions / Known limits

<!-- In the future, you can add assumptions and known limitations of this package. -->

## (Optional) Error detection and handling

<!-- In the future, you can add details about how this package detects and handles errors. -->

## (Optional) Performance characterization

<!-- In the future, you can add details about the performance of this package. -->

## (Optional) References/External links

<!-- In the future, you can add references and links to external code used in this package. -->

## (Optional) Future extensions / Unimplemented parts

<!-- In the future, you can add details about planned extensions or unimplemented parts of this package. -->
