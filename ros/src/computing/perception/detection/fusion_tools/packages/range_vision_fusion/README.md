ng# Range Vision Fusion

The Range Vision Fusion node will try match the objects detected on a range sensor, with the ones obtained from a vision detector. 
A match will be considered found if the 3D projection of the object overlaps at least 50% (configurable) over the 2D object detection.
The label from the 2D Image detector will be attached to the corresponding 3D Object. All the matched results will be published. 

## Requirements

### Input Topics
1. Camera intrinsics (`sensor_msgs/CameraInfo`)
1. Camera-LiDAR extrinsics (`tf`)
1. Object Detections results from a Vision Detector (`autoware_msgs/DetectedObjectArray`)
1. Object Detections results from a Range Detector (`autoware_msgs/DetectedObjectArray`)

### Output Topics
1. Fused Detected Objects (`autoware_msgs/DetectedObjectArray`) on the `/detection/combined_objects` topic.
1. Fused Detected Objects' boxes (`jsk_recognition_msgs/BoundingBoxArray`) on the `/detection/combined_objects_boxes` topic.
1. Fused Detected Objects' labels (`visualization_msgs/MarkerArray`) on the `/detection/combined_objects_labels` topic.

## Parameters

Launch file available parameters:

|Parameter| Type| Description|Default|
----------|-----|--------|---|
|`detected_objects_range`|*String* |Name of the `DetectedObjectArray` topic to subscribe containing the detections on 3D space.|`/detection/lidar_objects`|
|`detected_objects_vision`|*String*|Name of the `DetectedObjectArray` topic to subscribe containing the detections on 2D space.|`/detection/vision_objects`|
|`camera_info_src`|*String*|Name of the CameraInfo topic that contains the intrinsic matrix for the Image.|`/camera_info`|
|`sync_topics`|*Bool*|Sync detection topics.|`false`|
|`overlap_threshold`|*float*|A number between 0.1 and 1.0 representing the area of overlap between the detections.|`0.5`|

## Example of usage

1. Launch a ground filter algorithm from the `Points Preprocessor` in the **Sensing** tab. (adjust the parameters to your vehicle setup).
1. Launch Calibration Publisher with the intrinsic and extrinsic calibration between the camera and the range sensor.
1. Launch a Vision Detector in the Computing tab (this should be publishing by default `/detectoion/vision_objects`).
1. Launch a Lidar Detector in the Computing tab (this should be publishing by default `/detectoion/lidar_objects`).
1. Launch this node.
1. Launch `rviz`, and add the topics shown above in the Output section.

## Notes

Detection on Image space should be performed on a **Rectified** Image, otherwise projection will be incorrect.