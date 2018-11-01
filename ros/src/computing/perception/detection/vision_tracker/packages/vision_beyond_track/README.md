# Beyond Pixels based tracker

This node is based on the work depicted by Beyond Pixels: **Leveraging Geometry and Shape Cues for Online Multi-Object Tracking**.
Published on the *Proceedings of the IEEE International Conference on Robotics and Automation*.

### How to launch

Example, supposing default topic values. 

1. Launch CameraInfo publisher (Sensing Tab -> Calibration Publisher).

1. Launch the Image rectifier (`roslaunch image_processor image_rectifier.launch`).

1. Launch an image based object detector (yolo3, ssd, etc).

1. From a sourced terminal:

    - `roslaunch vision_beyond_track vision_beyond_track.launch`

or from Runtime Manager:

Computing Tab -> Detection/ vision_tracker -> `vision_beyond_track`

### Parameters

Launch file available parameters:

|Parameter| Type| Description|
----------|-----|--------
|`camera_info_src`|*String* |Camera intrinsics. Default `/camera_info`.|
|`objects_topic_src`|*String*|Image detections to track. Default `/detection/vision_objects`.|
|`camera_height`|*Double*|Camera Height in meters. Default `1.2`.|

### Subscribed topics

|Topic|Type|Objective|
------|----|---------
|`/camera_info`|`sensor_msgs/CameraInfo`|Camera intrinsics.|
|`/detection/vision_objects`|`autoware_msgs/DetectedObjectArray`|Obtain the rectangles of the detected objects on image.|

### Published topics

|Topic|Type|Objective|
------|----|---------
|`/detection/tracked_objects`|`autoware_msgs::DetectedObjectArray`|Contains the coordinates of the bounding boxes in image coordinates for the successfully tracked objects.|


### Video

[![Beyond Autoware](https://img.youtube.com/vi/KFfD3Mkkz4Y/0.jpg)](https://www.youtube.com/watch?v=KFfD3Mkkz4Y)
