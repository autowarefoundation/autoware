# ndt_matching_monitor

Simple health monitoring and reinitialization for `ndt_matching` localization node of
 [Autoware](https://github.com/CPFL/Autoware)

## Introduction
`ndt_matching` publishes the current robot/vehicle position in the `/ndt_pose` topic and statistics 
in the `/ndt_stat` topic, such as score, execution time. 

The score is the result of PCL's function `getFitnessScore()` which measures how well an input point cloud matches
 the reference pointcloud, in other words the alignment error between the input scan and the reference map.
 This value can be used to infer the reliability of NDT.

`ndt_matching_monitor` subscribes to `/ndt_stat` and `/ndt_pose` topics, and keeps a running average filter on 
the score value. When the filtered score is beyond some thresholds, `ndt_matching_monitor` will issue 
the `/initialpose` topic (same as rviz) using the last known "healty" pose to force `/ndt_matching` to initialize.

If a GNSS device is available, an automatic reinitialization will be triggered with it. `nmea2pose` node is required.
Otherwise, a halt in localization will be started. To reset the halted status, use *Initial Pose* tool in RVIZ. 

### How to launch
* From a sourced terminal:\
`roslaunch lidar_localizer ndt_matching_monitor.launch`

* From Runtime manager:\
Computing Tab -> lidar_localizer -> ndt_matching_monitor

### Parameters available

|Parameter| Type| Description|
----------|-----|--------
|`iteration_threshold_warn`|*integer* |Number of maximum iterations before notifying a warning. Default 10.|
|`iteration_threshold_stop`|*integer*|Number of maximum iterations before notifying a warning. Default 32. |
|`score_delta_threshold`|*double*|Difference between consecutive scores to consider a safe match. Default 14.0|
|`min_stable_samples`|*integer*|Minimum number of samples to start monitoring after a reset. Default 30|
|`fatal_time_threshold`|*double*|When no GNSS is available a prediction based on the previous samples will be employed. If reinitialization fails the algorithm will stop after n secs. Default 2|

### Subscribed topics

|Topic|Type|Objective|
------|----|---------
|`ndt_stat`|`autoware_msgs/ndt_stat`|Obtain NDT statistics: `score`, `iterations`.|
|`ndt_pose`|`geometry_msgs/PoseStamped`|Obtain pose as calculated by `ndt_matching_node`.|
|`initialpose`|`geometry_msgs/PoseWithCovarianceStamped`|Obtain pose from RVIZ to reinitialize in case of an unrecoverable error.|
|`gnss_pose`|`geometry_msgs/Pose`|If a GNSS device is available it will be used to automatically try to reset NDT, requires `nmea2tfpose`.|

### Published topics

|Topic|Type|Objective|
------|----|---------
|`initialpose`|`geometry_msgs/PoseWithCovarianceStamped`|Used to cause a reset or halt in `ndt_matching`.|
|`/ndt_monitor/ndt_info_text`|`jsk_rviz_plugins/OverlayText`|Publishes the text to be displayed in RVIZ with the help of `OverlayText` display type.|
|`/ndt_monitor/ndt_status`|`std_msgs/String`|Publishes the text for its later use in safety decision maker.|

## Contributing
`ndt_matching_monitor` is far from complete. 
For instance, the last known "healthy" pose stated above is just the last `/ndt_pose` value. 
A Kalman or particle filter might be used to improve this node. 
