# Detected Object Visualizer

This node interprets the output of each of the perception nodes and creates the corresponding rviz visualization.

## Parameters

Launch file available parameters:

|Parameter| Type| Description|
----------|-----|--------
|`marker_display_duration`|*Double*|Time in ms to display the marker. Default `0.1`.|
|`object_speed_threshold`|*Double* |Speed threshold to display. Default `0.1`.|
|`arrow_speed_threshold`|*Double*|Arrow speed threshold to display. Default `0.25`.|

### Subscribed topics

|Topic|Type|Objective|
------|----|---------
|`ROS_NAMESPACE/objects`|`autoware_msgs/DetectedObjectArray`|Objects Array topic to visualize|

### Published topics

|Topic|Type|Objective|
------|----|---------|
|`ROS_NAMESPACE/objects_labels`|visualization_msgs::MarkerArray|A Label indicating the class and info of the object|
|`ROS_NAMESPACE/objects_arrows`|visualization_msgs::MarkerArray|An arrow indicating the direction|
|`ROS_NAMESPACE/objects_hulls`|jsk_recognition_msgs::PolygonArray|Convex Hull, the containing polygon|
|`ROS_NAMESPACE/objects_boxes`|jsk_recognition_msgs::BoundingBoxArray|Bounding box containing the object|
|`ROS_NAMESPACE/objects_centroids`|visualization_msgs::MarkerArray|Sphere representing the centroid of the object in space|

## Notes
This node is already included in the perception nodes' launch file.

If you need to use it manually, be sure to add the `ROS_NAMESPACE` if using `rosrun`.

For the launch file case, add the following line to your launch file:
```xml
<node pkg="detected_objects_visualizer" type="visualize_detected_objects" name="AN_INSTANCENAME_01"
          output="screen" ns="ROS_NAMESPACE"/>
```

i.e. to visualize the output of the lidar tracker use `ROS_NAMESPACE=/detection/lidar_tracker` in rosrun or
`ns="/detection/lidar_tracker"` in the launch file.