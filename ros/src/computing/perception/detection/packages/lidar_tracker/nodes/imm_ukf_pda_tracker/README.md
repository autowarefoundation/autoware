# IMM-UKF-PDA Tracker

Autoware package based on IMM-UKF-PDA tracker.

* From a sourced terminal:

`roslaunch lidar_tracker imm_ukf_pda_tracker.launch`


* From Runtime Manager:

Computing Tab -> Detection/ lidar_detector -> `imm_ukf_pda_tracker`


### Reference
A. Arya Senna Abdul Rachman, 3D-LIDAR Multi Object Tracking for Autonomous Driving. 2017. [paper](https://repository.tudelft.nl/islandora/object/uuid:f536b829-42ae-41d5-968d-13bbaa4ec736)

M. Schreire, Bayesian environment representation, prediction, and criticality assessment for driver assistance systems. 2017. [paper](https://www.researchgate.net/publication/313463578_Bayesian_environment_representation_prediction_and_criticality_assessment_for_driver_assistance_systems)

### Requirements
* `eucledian_cluster` node.
* `ray_ground_filter` node.
* `/tf` topic. Below video is from Suginami data which contais /tf topic: (`autoware-20180205150908.bag`). You can download it from ROSBAG STORE for free. Otherwise, you need to do localization with a map to produce /tf topic from `velodyne` to `world`.
* `wayarea` info from vectormap if is possible.

### Parameters

Launch file available parameters for `imm_ukf_pda_tracker`

|Parameter| Type| Description|
----------|-----|--------
|`input topic`|*String* |Input topic(type: autoware_msgs::CloudClusterArray). Default `/cloud cluster`.|
|`output topic`|*String*|Output topic(type: autoware_msgs::CloudClusterArray). Default `/tracking_cluster_array`.|
|`pointcloud frame`|*String*|Pointcloud frame. Default `velodyne`.|
|`life time thres`|*Int*|The minimum frames for targets to be visualized. Default `8`.|
|`gating thres`|*Double*|The value of gate threshold for measurement validation. Default `9.22`.|
|`gate probability`|*Double*|The probability that the gate contains the true measurement. Default `0.99`.|
|`detection probability`|*Double*|The probability that a target is detected. Default `0.9`.|
|`static distance thres`|*Double*|The distance threshold for classifying static/dynamic. Default `3.0`.|




Launch file available parameters for `visualize_cloud_cluster`

|Parameter| Type| Description|
----------|-----|--------
|`input_topic`|*String* |Input topic(type: autoware_msgs::CloudClusterArray). Default `/tracking_cluster_array`.|
|`pointcloud frame`|*String*|Pointcloud frame. Default `velodyne`.|


### Subscribed topics
Node: imm_ukf_pda_tracker

|Topic|Type|Objective|
------|----|---------
|`/cloud_cluster`|`autoware_msgs::CloudClusterArray`|Segmented pointcloud from a clustering algorithm like eucledian cluster.|
|`/tf`|`tf`|Tracking objects in `world` coordinate.|

Node: visualize_cloud_cluster

|Topic|Type|Objective|
------|----|---------
|`/tracking_cluster_array`|`autoware_msgs::CloudClusterArray`|Segmented pointcloud with tracking info.|

### Published topics

Node: imm_ukf_pda_tracker

|Topic|Type|Objective|
------|----|---------
|`/tracking_cluster_array`|`autoware_msgs::CloudClusterArray`|Added info like velocity, yaw ,yaw_rate and static/dynamic class to segmented pointcloud.|

Node: visualize_cloud_cluster

|Topic|Type|Objective|
------|----|---------
|`/tracking_cluster_array/jsk_bb`|`jsk_recognition_msgs::BoundingBoxArray`|Visualze bounsing box nicely in rviz by JSK bounding box. Label contains information about static/dynamic class|
|`/tracking_cluster_array/velocity_arrow`|`visualization_msgs::Marker`|Visualize velocity and yaw of the targets.|
|`/tracking_cluster_array/target_id`|`visualization_msgs::Marker`|Visualize targets' id.|

### Node Graph

![node graph](./image/graph.png "node graph")

※ It is recommended to run eucledian cluster with vectormap if is possible. However, you can use the tracker without vectormap.

### Video

[![IMM UKF PDA lidar_tracker Autoware](https://img.youtube.com/vi/tKgDVsIfH-s/0.jpg)](https://youtu.be/tKgDVsIfH-s)
