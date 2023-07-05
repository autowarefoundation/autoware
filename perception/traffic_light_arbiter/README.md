# traffic_light_arbiter

## Purpose

This package receives traffic signals from perception and external (e.g., V2X) components and combines them using either a confidence-based or a external-preference based approach.

## TrafficLightArbiter

A node that merges traffic light/signal state from image recognition and external (e.g., V2X) systems to provide to a planning component.

### Inputs / Outputs

#### Input

| Name                             | Type                                              | Description                                              |
| -------------------------------- | ------------------------------------------------- | -------------------------------------------------------- |
| ~/sub/vector_map                 | autoware_auto_mapping_msgs::msg::HADMapBin        | The vector map to get valid traffic signal ids.          |
| ~/sub/perception_traffic_signals | autoware_perception_msgs::msg::TrafficSignalArray | The traffic signals from the image recognition pipeline. |
| ~/sub/external_traffic_signals   | autoware_perception_msgs::msg::TrafficSignalArray | The traffic signals from an external system.             |

#### Output

| Name                  | Type                                              | Description                      |
| --------------------- | ------------------------------------------------- | -------------------------------- |
| ~/pub/traffic_signals | autoware_perception_msgs::msg::TrafficSignalArray | The merged traffic signal state. |

## Parameters

### Core Parameters

| Name                        | Type   | Default Value | Description                                                                                                                      |
| --------------------------- | ------ | ------------- | -------------------------------------------------------------------------------------------------------------------------------- |
| `external_time_tolerance`   | double | 5.0           | The duration in seconds an external message is considered valid for merging                                                      |
| `perception_time_tolerance` | double | 1.0           | The duration in seconds a perception message is considered valid for merging                                                     |
| `external_priority`         | bool   | false         | Whether or not externals signals take precedence over perception-based ones. If false, the merging uses confidence as a criteria |
