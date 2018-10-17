# Naive Motion Prediction

Autoware package for motion prediction.

* From a sourced terminal:

`roslaunch naive_motion_predict naive_motion_predict.launch`


* From Runtime Manager:

Computing Tab -> Prediction/ Motion Predictor -> `naive_motion_predict`

### Requirements
* `ray_ground_filter` node.
* `eucledian_cluster` node.
* `/tf` topic.  `velodyne` to `world`.
* `imm_ukf_pda_tracker` node.

### Parameters

Launch file available parameters for `naive_motion_predict`

|Parameter| Type| Description|
----------|-----|--------
|`input topic`|*String* |Input topic(type: autoware_msgs::DetectedObjectArray). Default `/detection/lidar_tracker/objects`.|
|`interval_sec`|*Double*|Interval second for prediction. Default `9.22`.|
|`num_prediction`|*Int*|The number of prediction this node will make. Default `0.99`.|
|`sensor_height`|*Double*|Uses sensor height for visualized path's height. Default `0.9`.|

|


### Subscribed topics

|Topic|Type|Objective|
------|----|---------
|`/detection/lidar_tracker/objects`|`autoware_msgs::DetectedObjectArray`|The result topic from Autoware Detection stack|

### Published topics

|Topic|Type|Objective|
------|----|---------
|`/prediction/objects`|`autoware_msgs::DetectedObjectArray`|Added predicted objects to input data..|
|`/prediction/motion_predictor/path_markers`|`visualization_msgs::MarkerArray`|Visualzing predicted path in ros marker array|

### Video

[![Autoware: How to launch naive motion predict](https://img.youtube.com/vi/T6ediU0CHP0/0.jpg)](https://www.youtube.com/watch?v=T6ediU0CHP0&feature=youtu.be)
