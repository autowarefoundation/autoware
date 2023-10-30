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

| Name          | Type                                                  | Description      |
| ------------- | ----------------------------------------------------- | ---------------- |
| `~/input`     | `autoware_auto_perception_msgs::msg::DetectedObjects` | Detected objects |
| `/vector/map` | `autoware_auto_msgs::msg::HADMapBin`                  | Map data         |

### Output

| Name       | Type                                                 | Description     |
| ---------- | ---------------------------------------------------- | --------------- |
| `~/output` | `autoware_auto_perception_msgs::msg::TrackedObjects` | Tracked objects |

## Parameters

### Node Parameters

| Name                                 | Type   | Default Value               | Description                                                                                                     |
| ------------------------------------ | ------ | --------------------------- | --------------------------------------------------------------------------------------------------------------- |
| `publish_rate`                       | double | 10.0                        | The rate at which to publish the output messages                                                                |
| `world_frame_id`                     | string | "map"                       | The frame ID of the world coordinate system                                                                     |
| `enable_delay_compensation`          | bool   | false                       | Whether to enable delay compensation. If set to `true`, output topic is published by timer with `publish_rate`. |
| `tracking_config_directory`          | string | "./config/tracking/"        | The directory containing the tracking configuration files                                                       |
| `enable_logging`                     | bool   | false                       | Whether to enable logging                                                                                       |
| `logging_file_path`                  | string | "/tmp/association_log.json" | The path to the file where logs should be written                                                               |
| `tracker_lifetime`                   | double | 1.0                         | The lifetime of the tracker in seconds                                                                          |
| `use_distance_based_noise_filtering` | bool   | true                        | Whether to use distance based filtering                                                                         |
| `minimum_range_threshold`            | double | 70.0                        | Minimum distance threshold for filtering in meters                                                              |
| `use_map_based_noise_filtering`      | bool   | true                        | Whether to use map based filtering                                                                              |
| `max_distance_from_lane`             | double | 5.0                         | Maximum distance from lane for filtering in meters                                                              |
| `max_angle_diff_from_lane`           | double | 0.785398                    | Maximum angle difference from lane for filtering in radians                                                     |
| `max_lateral_velocity`               | double | 5.0                         | Maximum lateral velocity for filtering in m/s                                                                   |
| `can_assign_matrix`                  | array  |                             | An array of integers used in the data association algorithm                                                     |
| `max_dist_matrix`                    | array  |                             | An array of doubles used in the data association algorithm                                                      |
| `max_area_matrix`                    | array  |                             | An array of doubles used in the data association algorithm                                                      |
| `min_area_matrix`                    | array  |                             | An array of doubles used in the data association algorithm                                                      |
| `max_rad_matrix`                     | array  |                             | An array of doubles used in the data association algorithm                                                      |
| `min_iou_matrix`                     | array  |                             | An array of doubles used in the data association algorithm                                                      |

See more details in the [models.md](models.md).

### Tracker parameters

Currently, this package supports the following trackers:

- `linear_motion_tracker`
- `constant_turn_rate_motion_tracker`

Default settings for each tracker are defined in the [./config/tracking/](./config/tracking/), and described in [models.md](models.md).

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
