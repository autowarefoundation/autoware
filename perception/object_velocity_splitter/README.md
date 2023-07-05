# object_velocity_splitter

This package contains a object filter module for [autoware_auto_perception_msgs/msg/DetectedObject](https://gitlab.com/autowarefoundation/autoware.auto/autoware_auto_msgs/-/blob/master/autoware_auto_perception_msgs/msg/DetectedObject.idl).
This package can split DetectedObjects into two messages by object's speed.

## Input

| Name              | Type                                                  | Description          |
| ----------------- | ----------------------------------------------------- | -------------------- |
| `~/input/objects` | autoware_auto_perception_msgs/msg/DetectedObjects.msg | 3D detected objects. |

## Output

| Name                          | Type                                                  | Description             |
| ----------------------------- | ----------------------------------------------------- | ----------------------- |
| `~/output/low_speed_objects`  | autoware_auto_perception_msgs/msg/DetectedObjects.msg | Objects with low speed  |
| `~/output/high_speed_objects` | autoware_auto_perception_msgs/msg/DetectedObjects.msg | Objects with high speed |

## Parameters

| Name                 | Type   | Description                                         | Default value |
| :------------------- | :----- | :-------------------------------------------------- | :------------ |
| `velocity_threshold` | double | Velocity threshold parameter to split objects [m/s] | 3.0           |
