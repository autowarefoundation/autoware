# Voxel Grid Filter

Autoware package that downsampling of point cloud using 3D Voxel Grid.

### Requirements

* None

### How to launch

* From a sourced terminal:
    - `roslaunch points_downsampler points_downsample.launch node_name:=voxel_grid_filter`

* From Runtime Manager:

Sensing Tab -> Points Downsampler -> `voxel_grid_filter`
You can change the config, as well as other parameters, by clicking [app]

### Parameters

Launch file available parameters:

|Parameter| Type| Description|
----------|-----|--------
|`points_topic`|*String*|PointCloud source topic. Default `/points_raw`.|
|`voxel_leaf_size`|*Double*|Leaf Size. This parameter is used when `use_dynamic_leaf_size == False`.  Default `0.1` (mether).|
|`measurement_range`|*Double*|Maximum measurement range. Points far from the parameter are deleted. Default `200.0` (mether).|
|`use_dynamic_leaf_size`|*Bool*|If True, dynamically change the Leaf size so that the points size after downsampling falls within a certain range, using the following five parameters. Default `False`.|
|`voxel_leaf_size_step`|*Double*|Step size when looking for the optimum leaf size. Default `0.1` (mether).|
|`min_voxel_leaf_size`|*Double*|Minimum leaf size. Default `0.2` (mether).|
|`max_voxel_leaf_size`|*Double*|Maximum leaf size. Default `3.0` (mether).|
|`min_points_size`|*Double*|Minimum points size. Default `1500`.|
|`max_points_size`|*Double*|Maximum points size. Default `2000`.|

### Subscribed topics

|Topic|Type|Objective|
------|----|---------
|`/points_raw`|`sensor_msgs/PointCloud2`|PointCloud source topic.|
|`/config/voxel_grid_filter`|`autoware_msgs/ConfigVoxelGridFilter`|Configuration adjustment for threshold.|

### Published topics

|Topic|Type|Objective|
------|----|---------
|`/filtered_points`|`sensor_msgs/PointCloud2`|PointCloud after downsampling.|
|`/points_downsampler_info`|`points_downsampler/PointsDownsamplerInfo`|Points size after downsampling, execute time, etc.|

### Video

[![Voxel Grid Filter Autoware](https://img.youtube.com/vi/z-z0Is-_dEE/0.jpg)](https://www.youtube.com/watch?v=z-z0Is-_dEE)
