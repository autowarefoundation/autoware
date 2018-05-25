# LiDAR-Camera Fusion

## Pixel-Cloud fusion node

This node projects PointCloud to Image space, extracts RGB information from the Image, back-projects it to LiDAR space, and finally publishes a Colored PointCloud.

### Requirements

1. Camera intrinsics
1. Camera-LiDAR extrinsics
1. Image rectified being published


### How to launch

* From a sourced terminal:

`roslaunch pixel_cloud_fusion pixel_cloud_fusion.launch`

* From Runtime Manager:

Computing Tab -> Fusion -> pixel_cloud_fusion

### Parameters

Launch file available parameters:

|Parameter| Type| Description|Default|
----------|-----|--------|---|
|`points_src`|*String* |Name of the PointCloud topic to subscribe.|Default `points_raw`|
|`image_src`|*String*|Name of the Image topic to subscribe **NOTE: Must be a previously rectified image (check Autoware's `image_processor` or ROS `image_proc`.**|Default: `image_rectified`|
|`camera_info_src`|*String*|Name of the CameraInfo topic that contains the intrinsic matrix for the Image.|`camera_info`|

### Subscriptions/Publications


```
Node [/pixel_cloud_fusion]
Publications: 
 * /points_fused [sensor_msgs/PointCloud2]

Subscriptions: 
 * /image_raw [sensor_msgs/Image]
 * /points_raw [sensor_msgs/PointCloud2]
 * /camera_info [sensor_msgs/CameraInfo]
 * /tf [tf2_msgs/TFMessage]
```

[![Demo Movie](http://img.youtube.com/vi/KhcEpT_3GB4/mqdefault.jpg)](https://www.youtube.com/watch?v=KhcEpT_3GB4)