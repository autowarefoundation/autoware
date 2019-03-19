# Compare Map Filter

Autoware package that compare the LiDAR PointCloud and PointCloud Map and then extract (or eliminate) coincident points.

### Requirements

* PointCloud Map with extremely few unnecessary PointCloud (people, cars, etc.).

### How to launch

* From a sourced terminal:
    - `roslaunch points_preprocessor compare_map_filter.launch`

* From Runtime Manager:

Sensing Tab -> Points Preprocessor -> `compare_map_filter`
You can change the config, as well as other parameters, by clicking [app]

### Parameters

Launch file available parameters:

|Parameter| Type| Description|
----------|-----|--------
|`input_point_topic`|*String*|PointCloud source topic. Default `/points_raw`.|
|`input_map_topic`|*String*|PointCloud Map topic. Default `/points_map`.|
|`output_match_topic`|*String*|Comparing LiDAR PointCloud and PointCloud Map, topic of the PointCloud which matched. Default `/match_points`.|
|`output_unmatch_topic`|*String*|Comparing LiDAR PointCloud and PointCloud Map, topic of the PointCloud which unmatched. Default `/unmatch_points`.|
|`distance_threshold`|*Double*|Threshold for comparing LiDAR PointCloud and PointCloud Map. Euclidean distance (mether).  Default `0.3`.|
|`min_clipping_height`|*Double*|Remove the points where the height is lower than the threshold. (Based on sensor coordinates). Default `-2.0`.|
|`max_clipping_height`|*Double*|Remove the points where the height is higher than the threshold. (Based on sensor coordinates). Default `0.5`.|

### Subscribed topics

|Topic|Type|Objective|
------|----|---------
|`/points_raw`|`sensor_msgs/PointCloud2`|PointCloud source topic.|
|`/points_map`|`sensor_msgs/PointCloud2`|PointCloud Map topic.|
|`/config/compare_map_filter`|`autoware_msgs/ConfigCompareMapFilter`|Configuration adjustment for threshold.|
|`/tf`|TF|sensor frame <-> map frame|

### Published topics

|Topic|Type|Objective|
------|----|---------
|`/match_points`|`sensor_msgs/PointCloud2`|Comparing LiDAR PointCloud and PointCloud Map, topic of the PointCloud which matched.|
|`/unmatch_points`|`sensor_msgs/PointCloud2`|Comparing LiDAR PointCloud and PointCloud Map, topic of the PointCloud which unmatched.|

### Video

[![Compare Map Filter Autoware](https://img.youtube.com/vi/7nK6JrI7TAI/0.jpg)](https://www.youtube.com/watch?v=7nK6JrI7TAI)
