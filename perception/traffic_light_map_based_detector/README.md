# The `traffic_light_map_based_detector` Package

## Overview

`traffic_light_map_based_detector` calculates where the traffic lights will appear in the image based on the HD map.

Calibration and vibration errors can be entered as parameters, and the size of the detected RegionOfInterest will change according to the error.

![traffic_light_map_based_detector_result](./docs/traffic_light_map_based_detector_result.svg)

If the node receives route information, it only looks at traffic lights on that route.
If the node receives no route information, it looks at a radius of 200 meters and the angle between the traffic light and the camera is less than 40 degrees.

## Input topics

| Name                 | Type                                  | Description             |
| -------------------- | ------------------------------------- | ----------------------- |
| `~input/vector_map`  | autoware_auto_mapping_msgs::HADMapBin | vector map              |
| `~input/camera_info` | sensor_msgs::CameraInfo               | target camera parameter |
| `~input/route`       | autoware_planning_msgs::LaneletRoute  | optional: route         |

## Output topics

| Name             | Type                                                | Description                                                          |
| ---------------- | --------------------------------------------------- | -------------------------------------------------------------------- |
| `~output/rois`   | autoware_auto_perception_msgs::TrafficLightRoiArray | location of traffic lights in image corresponding to the camera info |
| `~debug/markers` | visualization_msgs::MarkerArray                     | visualization to debug                                               |

## Node parameters

| Parameter              | Type   | Description                                                 |
| ---------------------- | ------ | ----------------------------------------------------------- |
| `max_vibration_pitch`  | double | Maximum error in pitch direction. If -5~+5, it will be 10.  |
| `max_vibration_yaw`    | double | Maximum error in yaw direction. If -5~+5, it will be 10.    |
| `max_vibration_height` | double | Maximum error in height direction. If -5~+5, it will be 10. |
| `max_vibration_width`  | double | Maximum error in width direction. If -5~+5, it will be 10.  |
| `max_vibration_depth`  | double | Maximum error in depth direction. If -5~+5, it will be 10.  |
