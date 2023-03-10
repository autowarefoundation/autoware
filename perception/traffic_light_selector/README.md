# traffic_light_selector

## Purpose

This package receives multiple traffic light/signal states and outputs a single traffic signal state for use by the planning component.

## TrafficLightSelector

A node that merges traffic light/signal state from image recognition and V2X to provide a planning component.
It's currently a provisional implementation.

### Inputs / Outputs

#### Input

| Name                 | Type                                           | Description                                       |
| -------------------- | ---------------------------------------------- | ------------------------------------------------- |
| ~/sub/vector_map     | autoware_auto_mapping_msgs/msg/HADMapBin       | The vector map to get traffic light id relations. |
| ~/sub/traffic_lights | autoware_perception_msgs/msg/TrafficLightArray | The traffic light state from image recognition.   |

#### Output

| Name                  | Type                                            | Description                      |
| --------------------- | ----------------------------------------------- | -------------------------------- |
| ~/pub/traffic_signals | autoware_perception_msgs/msg/TrafficSignalArray | The merged traffic signal state. |

## TrafficLightConverter

A temporary node that converts old message type to new message type.

### Inputs / Outputs

#### Input

| Name                 | Type                                                 | Description                    |
| -------------------- | ---------------------------------------------------- | ------------------------------ |
| ~/sub/traffic_lights | autoware_auto_perception_msgs/msg/TrafficSignalArray | The state in old message type. |

#### Output

| Name                 | Type                                           | Description                    |
| -------------------- | ---------------------------------------------- | ------------------------------ |
| ~/pub/traffic_lights | autoware_perception_msgs/msg/TrafficLightArray | The state in new message type. |
