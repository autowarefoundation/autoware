# The `traffic_light_multi_camera_fusion` Package

## Overview

`traffic_light_multi_camera_fusion` performs traffic light signal fusion which can be summarized as the following two tasks:

1. Multi-Camera-Fusion: performed on single traffic light signal detected by different cameras.
2. Group-Fusion: performed on traffic light signals within the same group, which means traffic lights sharing the same regulatory element id defined in lanelet2 map.

## Input topics

For every camera, the following three topics are subscribed:

| Name                                   | Type                                           | Description                                         |
| -------------------------------------- | ---------------------------------------------- | --------------------------------------------------- |
| `~/<camera_namespace>/camera_info`     | sensor_msgs::CameraInfo                        | camera info from traffic_light_map_based_detector   |
| `~/<camera_namespace>/rois`            | tier4_perception_msgs::TrafficLightRoiArray    | detection roi from traffic_light_fine_detector      |
| `~/<camera_namespace>/traffic_signals` | tier4_perception_msgs::TrafficLightSignalArray | classification result from traffic_light_classifier |

You don't need to configure these topics manually. Just provide the `camera_namespaces` parameter and the node will automatically extract the `<camera_namespace>` and create the subscribers.

## Output topics

| Name                       | Type                                              | Description                        |
| -------------------------- | ------------------------------------------------- | ---------------------------------- |
| `~/output/traffic_signals` | autoware_perception_msgs::TrafficLightSignalArray | traffic light signal fusion result |

## Node parameters

| Parameter              | Type            | Description                                      |
| ---------------------- | --------------- | ------------------------------------------------ |
| `camera_namespaces`    | vector\<string> | Camera Namespaces to be fused                    |
| `message_lifespan`     | double          | The maximum timestamp span to be fused           |
| `approximate_sync`     | bool            | Whether work in Approximate Synchronization Mode |
| `perform_group_fusion` | bool            | Whether perform Group Fusion                     |
