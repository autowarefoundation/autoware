# Naive-L-Shape fitting


* From a sourced terminal:

`roslaunch lidar_naive_l_shape_detect lidar_naive_l_shape_detect.launch`


* From Runtime Manager:

Computing Tab -> Detection/ lidar_detector -> `lidar_naive_l_shape_detect`


### Reference
A. Arya Senna Abdul Rachman, 3D-LIDAR Multi Object Tracking for Autonomous Driving. 2017. [paper](https://repository.tudelft.nl/islandora/object/uuid:f536b829-42ae-41d5-968d-13bbaa4ec736

### Requirements
* `lidar_eucledian_cluster_detect` node.

### Parameters

Launch file available parameters for `lidar_naive_l_shape_detect`

|Parameter| Type| Description|
----------|-----|--------
|`input topic`|*String* |Input topic(type: autoware_msgs::DetectedObjectArray). Default `/detection/lidar_objects`.|
|`output topic`|*String*|Output topic(type: autoware_msgs::DetectedObjectArray). Default `/detection/lidar_objects/l_shaped`.|
|`random_ponts`|*int*|Number of random sampling points. Default `80`.|
|`slope_dist_thres`|*float*|Threshold for applying L-shape fitting. Default `2.0`.|
|`num_points_thres`|*int*|Threshold for applying L-shape fitting.  Default `10`.|
|`sensor_height`|*float*|Lidar height from base_link. Default `2.3`.|


### Subscribed topics

|Topic|Type|Objective|
------|----|---------
|`/detection/lidar_objects`|`autoware_msgs::DetectedObjectArray`|Segmented pointcloud from a clustering algorithm like eucledian cluster.|

### Published topics

|Topic|Type|Objective|
------|----|---------
|`/detection/lidar_objects/l_shaped`|`autoware_msgs::DetectedObjectArray`|L-shape fitting pose and dimension will be published.|


### Video

[![Autoware: naive L shape fitting](https://img.youtube.com/vi/Zd37mE7sXyI/0.jpg)](https://www.youtube.com/watch?v=Zd37mE7sXyI)
