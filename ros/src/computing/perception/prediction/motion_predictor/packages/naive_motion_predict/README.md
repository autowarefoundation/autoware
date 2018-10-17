# Naive Motion Prediction

Autoware package for motion prediction.

* From a sourced terminal:

`roslaunch naive_motion_predict naive_motion_predict.launch`


* From Runtime Manager:

Computing Tab -> Prediction/ Motion Predictor -> `naive_motion_predict`

### Requirements
* `ray_ground_filter` node.
* `eucledian_cluster` node.
* `/tf` topic. Below video is from Suginami data which contais /tf topic: (`autoware-20180205150908.bag`). You can download it from ROSBAG STORE for free. Otherwise, you need to do localization with a map to produce /tf topic from `velodyne` to `world`.
* `imm_ukf_pda_tracker` node.

### Parameters

Launch file available parameters for `naive_motion_predict`

|Parameter| Type| Description|
----------|-----|--------
|`input topic`|*String* |Input topic(type: autoware_msgs::CloudClusterArray). Default `/cloud cluster`.|
|`output topic`|*String*|Output topic(type: autoware_msgs::CloudClusterArray). Default `/tracking_cluster_array`.|
|`pointcloud frame`|*String*|Pointcloud frame. Default `velodyne`.|
|`life time thres`|*Int*|The minimum frames for targets to be visualized. Default `8`.|
|`gating thres`|*Double*|The value of gate threshold for measurement validation. Default `9.22`.|
|`gate probability`|*Double*|The probability that the gate contains the true measurement. Default `0.99`.|
|`detection probability`|*Double*|The probability that a target is detected. Default `0.9`.|
|`distance thres`|*Double*|The distance threshold for associating bounding box over frames. Default `100`.|
|`static distance thres`|*Double*|The distance threshold for classifying static/dynamic. Default `3.0`.|




Launch file available parameters for `visualize_detected_objects`

|Parameter| Type| Description|
----------|-----|--------
|`input_topic`|*String* |Input topic(type: autoware_msgs::DetectedObjectArray). Default `/detection/objects`.|
|`internal_sec`|*double*|With this interval(unit: sec), this node makes prediction Default `0.1`.|
|`num_prediction`|*int*|Number of predictions this node makes. Default `20`.|
|`sensor_height`|*double*|Used for visualizing predicted path. Default `2.0`.|


### Subscribed topics

|Topic|Type|Objective|
------|----|---------
|`/detection/objects`|`autoware_msgs::DetectedObjectArray`|The result topic from Autoware Detection stack|

### Published topics

|Topic|Type|Objective|
------|----|---------
|`/prediction/objects`|`autoware_msgs::DetectedObjectArray`|Added predicted objects to input data..|
|`/prediction/motion_predictor/path_markers`|`visualization_msgs::MarkerArray`|Visualzing predicted path in ros marker array|
