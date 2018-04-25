# IMM-UKF-PDA Tracker

Autoware package based on IMM-UKF-PDA tracker.


* From a sourced terminal:

`roslaunch lidar_tracker imm_ukf_pda_tracker.launch`

`roslaunch lidar_tracker visualize_cloud_cluster.launch`

* From Runtime Manager:

Computing Tab -> Detection/ lidar_detector -> `imm_ukf_pda_tracker`

Computing Tab -> Detection/ lidar_detector -> `visualize_cloud_cluster`

### Parameters

Launch file available parameters(imm_ukf_pda_tracker):

|Parameter| Type| Description|
----------|-----|--------
|`input_topic`|*String* |Input topic(type: autoware_msgs::CloudClusterArray). Default `/cloud_cluster`.|
|`output_topic`|*String*|Output topic(type: autoware_msgs::CloudClusterArray). Default `/tracking_cluster_array`.|
|`life_time_thres_`|*Int*|The minimum frames for targets to be visualized. Default `8`.|

Launch file available parameters(visualize_cloud_cluster):

|Parameter| Type| Description|
----------|-----|--------
|`input_topic`|*String* |Input topic(type: autoware_msgs::CloudClusterArray). Default `/tracking_cluster_array`.|


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

### Node Graph

![node graph](./image/graph.png "node graph")

※ It is recommended to run eucledian cluster with vectormap if is possible. However, you can use the tracker without vectormap.

### Video

[![IMM UKF PDA lidar_tracker Autoware](https://img.youtube.com/vi/tKgDVsIfH-s/0.jpg)](https://youtu.be/tKgDVsIfH-s)
