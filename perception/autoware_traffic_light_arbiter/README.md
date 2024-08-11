# autoware_traffic_light_arbiter

## Purpose

This package receives traffic signals from perception and external (e.g., V2X) components and combines them using either a confidence-based or a external-preference based approach.

## TrafficLightArbiter

A node that merges traffic light/signal state from image recognition and external (e.g., V2X) systems to provide to a planning component.

### Signal Match Validator

When `enable_signal_matching` is set to true, this node validates the match between perception signals and external signals.
The table below outlines how the matching process determines the output based on the combination of perception and external signal colors. Each cell represents the outcome when a specific color from a perception signal (columns) intersects with a color from an external signal (rows).

| External \ Perception | RED     | AMBER   | GREEN   | UNKNOWN | Not Received |
| --------------------- | ------- | ------- | ------- | ------- | ------------ |
| RED                   | RED     | UNKNOWN | UNKNOWN | UNKNOWN | UNKNOWN      |
| AMBER                 | UNKNOWN | AMBER   | UNKNOWN | UNKNOWN | UNKNOWN      |
| GREEN                 | UNKNOWN | UNKNOWN | GREEN   | UNKNOWN | UNKNOWN      |
| UNKNOWN               | UNKNOWN | UNKNOWN | UNKNOWN | UNKNOWN | UNKNOWN      |
| Not Received          | UNKNOWN | UNKNOWN | UNKNOWN | UNKNOWN | UNKNOWN      |

### Inputs / Outputs

#### Input

| Name                             | Type                                                  | Description                                              |
| -------------------------------- | ----------------------------------------------------- | -------------------------------------------------------- |
| ~/sub/vector_map                 | autoware_map_msgs::msg::LaneletMapBin                 | The vector map to get valid traffic signal ids.          |
| ~/sub/perception_traffic_signals | autoware_perception_msgs::msg::TrafficLightGroupArray | The traffic signals from the image recognition pipeline. |
| ~/sub/external_traffic_signals   | autoware_perception_msgs::msg::TrafficLightGroupArray | The traffic signals from an external system.             |

#### Output

| Name                  | Type                                                  | Description                      |
| --------------------- | ----------------------------------------------------- | -------------------------------- |
| ~/pub/traffic_signals | autoware_perception_msgs::msg::TrafficLightGroupArray | The merged traffic signal state. |

## Parameters

### Core Parameters

| Name                        | Type   | Default Value | Description                                                                                                                                                                    |
| --------------------------- | ------ | ------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------ |
| `external_time_tolerance`   | double | 5.0           | The duration in seconds an external message is considered valid for merging                                                                                                    |
| `perception_time_tolerance` | double | 1.0           | The duration in seconds a perception message is considered valid for merging                                                                                                   |
| `external_priority`         | bool   | false         | Whether or not externals signals take precedence over perception-based ones. If false, the merging uses confidence as a criteria                                               |
| `enable_signal_matching`    | bool   | false         | Decide whether to validate the match between perception signals and external signals. If set to true, verify that the colors match and only publish them if they are identical |
