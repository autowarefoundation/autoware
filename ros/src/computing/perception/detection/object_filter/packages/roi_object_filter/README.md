# ROI Object Filter

The ROI object filter node uses information from the ADAS map to remove the detected objects outside of the drivable area.

## Requirements

1. PointCloud map.
1. ADAS map with drivable area included (`wayarea).
1. OccupancyGrid in GridMap format. 
1. NDT localization (TF tree properly constructed).

### Input Topics
1. GridMap containing the drivable area occupancy grid (`grid_map_msgs/GridMap`). Published by `wayarea2grid` node in the semantics package.
1. Object Detection results to filter (`autoware_msgs/DetectedObjectArray`). Published by an object detector.

### Output Topics
1. Objects with the `valid` field modified according to the ROI, including those in the list of exceptions (`autoware_msgs/DetectedObjectArray`) on the `/detection/object_filter/objects` topic.

## Parameters

Available parameters:

|Parameter| Type| Description|Default|
----------|-----|--------|---|
|`wayarea_gridmap_topic`|*String*|Topic name containing the GridMap with the drivable area occupancy grid.|`grid_map_wayarea`|
|`detection_topic`|*String*|Name of the `DetectedObjectArray` topic to subscribe containing the detections to filter.|`/detection/lidar_detector/objects`|
|`wayarea_no_road_value`|*Int*|Occupancy Grid integer value of the non drivable regions.|`255`|
|`sync_topics`|*Bool*|Whether or not to Sync detection topics using ROS filters.|`false`|
|`wayarea_gridmap_layer`|*String*|Name of the layer in the GridMap containing the drivable area occupancy grid.|`wayarea`|
|`exception_list`|*String Array*|A list of the object to be kept even when found outside the ROI.|`[person, bicycle]`|

## Usage example

1. Launch a ground filter algorithm from the `Points Preprocessor` section in the **Sensing** tab. (adjust the parameters to your vehicle setup).
1. Launch a Lidar Detector from the Computing tab or fusion.
1. Launch NDT localization with the Map and VectorMap being published.
1. Launch `wayarea2grid` from the Semantics package in the  *Computing*** tab.
1. Launch this node.

## Notes

* VectorMap MUST include the drivable area (i.e. `wayarea.csv`)