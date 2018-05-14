# tl_switch

## Introduction
The node uses traffic signal obtained by image recognition and that recieved from external systems, for example AMS, and switch traffic signals safely. If either is red, it turns red. Also, timeout is set for external system information.

### How to launch
* From a sourced terminal:

    `roslaunch trafficlight_recognizer traffic_light_recognition.launch`

* From Runtime manager:

    Computing Tab -> region_tlr

### Published topics
|Topic|Type|Objective|
------|----|---------
|`/camera_light_color`|`autoware_msgs/traffic_light`|Subscribe traffic light color obtained from camera image.|
|`/ams_light_color`|`autoware_msgs/traffic_light`| Subscribe traffic light color recieved from external systems.|

### Subscribed topics
|Topic|Type|Objective|
------|----|---------
|`/light_color`|`autoware_msgs/traffic_light`|Publishes traffic light color for planning.|
