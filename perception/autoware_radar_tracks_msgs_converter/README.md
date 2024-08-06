# radar_tracks_msgs_converter

This package converts from [radar_msgs/msg/RadarTracks](https://github.com/ros-perception/radar_msgs/blob/ros2/msg/RadarTracks.msg) into [autoware_perception_msgs/msg/DetectedObject](https://github.com/autowarefoundation/autoware_msgs/tree/main/autoware_perception_msgs/msg/DetectedObject.msg) and [autoware_perception_msgs/msg/TrackedObject](https://github.com/autowarefoundation/autoware_msgs/tree/main/autoware_perception_msgs/msg/TrackedObject.msg).

- Calculation cost is O(n).
  - n: The number of radar objects

## Design

### Background

Autoware uses [radar_msgs/msg/RadarTracks.msg](https://github.com/ros-perception/radar_msgs/blob/ros2/msg/RadarTracks.msg) as radar objects input data.
To use radar objects data for Autoware perception module easily, `radar_tracks_msgs_converter` converts message type from `radar_msgs/msg/RadarTracks.msg` to `autoware_perception_msgs/msg/DetectedObject`.
In addition, because many detection module have an assumption on base_link frame, `radar_tracks_msgs_converter` provide the functions of transform frame_id.

### Note

`Radar_tracks_msgs_converter` converts the label from `radar_msgs/msg/RadarTrack.msg` to Autoware label.
Label id is defined as below.

|            | RadarTrack | Autoware |
| ---------- | ---------- | -------- |
| UNKNOWN    | 32000      | 0        |
| CAR        | 32001      | 1        |
| TRUCK      | 32002      | 2        |
| BUS        | 32003      | 3        |
| TRAILER    | 32004      | 4        |
| MOTORCYCLE | 32005      | 5        |
| BICYCLE    | 32006      | 6        |
| PEDESTRIAN | 32007      | 7        |

Additional vendor-specific classifications are permitted starting from 32000 in [radar_msgs/msg/RadarTrack.msg](https://github.com/ros-perception/radar_msgs/blob/ros2/msg/RadarTrack.msg).
Autoware objects label is defined in [ObjectClassification](https://github.com/autowarefoundation/autoware_msgs/tree/main/autoware_perception_msgs/msg/ObjectClassification.msg)

## Interface

### Input

- `~/input/radar_objects` (`radar_msgs/msg/RadarTracks.msg`)
  - Input radar topic
- `~/input/odometry` (`nav_msgs/msg/Odometry.msg`)
  - Ego vehicle odometry topic

### Output

- `~/output/radar_detected_objects` (`autoware_perception_msgs/msg/DetectedObject.idl`)
  - DetectedObject topic converted to Autoware message.
  - This is used for radar sensor fusion detection and radar detection.
- `~/output/radar_tracked_objects` (`autoware_perception_msgs/msg/TrackedObject.idl`)
  - TrackedObject topic converted to Autoware message.
  - This is used for tracking layer sensor fusion.

### Parameters

#### Parameter Summary

{{ json_to_markdown("perception/autoware_radar_tracks_msgs_converter/schema/radar_tracks_msgs_converter.schema.json") }}

#### Parameter Description

- `update_rate_hz` (double) [hz]
  - Default parameter is 20.0

This parameter is update rate for the `onTimer` function.
This parameter should be same as the frame rate of input topics.

- `new_frame_id` (string)
  - Default parameter is "base_link"

This parameter is the header frame_id of the output topic.

- `use_twist_compensation` (bool)
  - Default parameter is "true"

This parameter is the flag to use the compensation to linear of ego vehicle's twist.
If the parameter is true, then the twist of the output objects' topic is compensated by the ego vehicle linear motion.

- `use_twist_yaw_compensation` (bool)
  - Default parameter is "false"

This parameter is the flag to use the compensation to yaw rotation of ego vehicle's twist.
If the parameter is true, then the ego motion compensation will also consider yaw motion of the ego vehicle.

- `static_object_speed_threshold` (float) [m/s]
  - Default parameter is 1.0

This parameter is the threshold to determine the flag `is_stationary`.
If the velocity is lower than this parameter, the flag `is_stationary` of DetectedObject is set to `true` and dealt as a static object.
