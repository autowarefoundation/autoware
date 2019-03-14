# Shape Estimation

L-shape fitting implementation of the paper:
```
@conference{Zhang-2017-26536,
author = {Xiao Zhang and Wenda Xu and Chiyu Dong and John M. Dolan},
title = {Efficient L-Shape Fitting for Vehicle Detection Using Laser Scanners},
booktitle = {2017 IEEE Intelligent Vehicles Symposium},
year = {2017},
month = {June},
keywords = {autonomous driving, laser scanner, perception, segmentation},
} 
```

## How to launch

### From RTM
Computing tab -> Detection -> lidar_detector -> lidar_shape_estimation

Configure parameters using the `[app]` button.

### From the command line
From a sourced command line:
`roslaunch lidar_shape_estimation shape_estimation_clustering.launch`

Launch files also include the visualization node.

## Requirements

1. LiDAR data segmented. 
1. Objects 

## Parameters

|Parameter| Type| Description|Default|
----------|-----|--------|---|
|`input`|*String*|Topic name containing the objects detected by the Lidar in 3D space.|`/detection/lidar_detector/objects`|
|`output`|*String*|Topic name containing the objects with the shape estimated in 3D space.|`/detection/lidar_shape_estimation/objects`|

## Usage example

1. Launch a ground filter algorithm from the `Points Preprocessor` section in the **Sensing** tab. (adjust the parameters to your vehicle setup).
1. Launch a Lidar Detector from the Computing tab.
1. Launch this node.

## Node info

```
Node [/lidar_shape_estimation]
Publications: 
 * /detection/shape_estimation/objects [autoware_msgs/DetectedObjectArray]

Subscriptions: 
 * /detection/lidar_detector/objects [autoware_msgs/DetectedObjectArray]
 
-------------------------
Node [/detection/shape_estimation/shape_estimation_visualization]
Publications: 
 * /detection/shape_estimation/objects_markers [visualization_msgs/MarkerArray]

Subscriptions: 
 * /detection/shape_estimation/objects [autoware_msgs/DetectedObjectArray]
```