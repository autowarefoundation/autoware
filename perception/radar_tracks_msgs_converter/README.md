# radar_tracks_msgs_converter

This package converts from [radar_msgs/msg/RadarTracks](https://github.com/ros-perception/radar_msgs/blob/ros2/msg/RadarTracks.msg) into [autoware_auto_perception_msgs/msg/DetectedObject](https://gitlab.com/autowarefoundation/autoware.auto/autoware_auto_msgs/-/blob/master/autoware_auto_perception_msgs/msg/DetectedObject.idl) and [autoware_auto_perception_msgs/msg/TrackedObject](https://gitlab.com/autowarefoundation/autoware.auto/autoware_auto_msgs/-/blob/master/autoware_auto_perception_msgs/msg/TrackedObject.idl).

- Calculation cost is O(n).
  - n: The number of radar objects

## Design

### Input / Output

- Input
  - `~/input/radar_objects` (radar_msgs/msg/RadarTracks.msg): Input radar topic
  - `~/input/odometry` (nav_msgs/msg/Odometry.msg): Ego vehicle odometry topic
- Output
  - `~/output/radar_detected_objects` (autoware_auto_perception_msgs/msg/DetectedObject.idl): The topic converted to Autoware's message. This is used for radar sensor fusion detection and radar detection.
  - `~/output/radar_tracked_objects` (autoware_auto_perception_msgs/msg/TrackedObject.idl): The topic converted to Autoware's message. This is used for tracking layer sensor fusion.

### Parameters

- `update_rate_hz` (double): The update rate [hz].
  - Default parameter is 20.0
- `new_frame_id` (string): The header frame of the output topic.
  - Default parameter is "base_link"
- `use_twist_compensation` (bool): If the parameter is true, then the twist of the output objects' topic is compensated by ego vehicle motion.
  - Default parameter is "false"
- `use_twist_yaw_compensation` (bool): If the parameter is true, then the ego motion compensation will also consider yaw motion of the ego vehicle.
  - Default parameter is "false"
- `static_object_speed_threshold` (float): Specify the threshold for static object speed which determines the flag `is_stationary` [m/s].
  - Default parameter is 1.0

## Note

This package convert the label from `radar_msgs/msg/RadarTrack.msg` to Autoware label.
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

- [radar_msgs/msg/RadarTrack.msg](https://github.com/ros-perception/radar_msgs/blob/ros2/msg/RadarTrack.msg): additional vendor-specific classifications are permitted starting from 32000.

- [Autoware objects label](https://gitlab.com/autowarefoundation/autoware.auto/autoware_auto_msgs/-/blob/master/autoware_auto_perception_msgs/msg/ObjectClassification.idl)
