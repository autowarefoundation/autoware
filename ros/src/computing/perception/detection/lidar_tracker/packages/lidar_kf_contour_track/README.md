# KF based Object Tracker  

## kf contour tracker 

This nodes contains three options of tracking 
- Association only 
- Simple KF tracking
- Contour area plust memory tracker (divid the horizon into contours and associate memory for each circle, which represent the maximum life time of each object) 

### Outputs
This tracker output (pose, heading, velocity) in global coordinates. 

### Object Filtering 
It can filter detected objects by size or/and vector map lanes proximity.

### Options
It tracks either OpenPlanner simulted vehicles or live detection cluster from  	"lidar_euclidean_cluster_detect" 
It can simulated frame by frame testing with fixed time intervals 0.1 second. 

### Requirements

1. cloud_clusters 
1. vector map, in case of using vector map object filtering (to disable using vector map set "vector map filter distance" to 0)
1. op percepion simulator, in case of simulation, also vector map filtering should be disabled. 

### How to launch

* From a sourced terminal:

`roslaunch lidar_kf_contour_track lidar_kf_contour_track.launch`

* From Runtime Manager:

Computing Tab -> Detection -> lidar_tracker -> lidar_kf_contour_tracker

### Subscriptions/Publications


```
Publications: 
 * /tracked_objects [autoware_msgs::DetectedObjectArray]
 * /detected_polygons [visualization_msgs::MarkerArray]
 * /op_planner_tracked_boxes [jsk_recognition_msgs::BoundingBoxArray]

Subscriptions: 
 * /cloud_cluster [autoware_msgs::CloudClusterArray]
 * /current_pose [geometry_msgs::PoseStamped]
 * /vector_map_info/* 
```

[Demo Movie](https://youtu.be/BS5nLtBsXPE)
