# Detected Object Visualizer

This node interprets the output of each of the perception nodes and creates the corresponding rviz visualization.

## Parameters

Launch file available parameters:

|Parameter| Type| Description|
----------|-----|--------
|`marker_display_duration`|*Double*|Time in ms to display the marker. Default `0.1`.|
|`object_speed_threshold`|*Double* |Speed threshold to display. Default `0.1`.|
|`arrow_speed_threshold`|*Double*|Arrow speed threshold to display. Default `0.25`.|
|`label_color`|*Array*|Four double values (RGBA), representing the color of the object. Default `[255,255,255,1]`.|
|`arrow_color`|*Array*|Four double values (RGBA), representing the color of the object. Default `[0,255,0,1,0.8]`.|
|`hull_color`|*Array*|Four double values (RGBA), representing the color of the object. Default `[51,204,51,0.8]`.|
|`box_color`|*Array*|Four double values (RGBA), representing the color of the object. Default `[51,128,204,0.8]`.|
|`centroid_color`|*Array*|Four double values (RGBA), representing the color of the object. Default `[77,121,255,0.8]`.|


### Subscribed topics

|Topic|Type|Objective|
------|----|---------
|`ROS_NAMESPACE/objects`|`autoware_msgs/DetectedObjectArray`|Objects Array topic to visualize|

### Published topics

|Topic|Type|Objective|
------|----|---------|
|`ROS_NAMESPACE/objects_markers`|visualization_msgs::MarkerArray|A Label indicating the class and info of the object|

The message includes each of the following namespaces:

|Namespace|Objective|
|------|---------|
|`ROS_NAMESPACE/objects_arrows`|An arrow indicating the direction (`Marker::ARROW`)|
|`ROS_NAMESPACE/objects_hulls`|Convex Hull, the containing polygon (`Marker::LINE_STRIP`)|
|`ROS_NAMESPACE/objects_boxes`|Bounding box containing the object (`Marker::CUBE`)|
|`ROS_NAMESPACE/objects_centroids`|Sphere representing the centroid of the object in space (`Marker::SPHERE`)|
|`ROS_NAMESPACE/objects_mdoels`|Model representing the object in space (`Marker::MESH_RESOURCE`)|

## Notes
This node is already included in the perception nodes' launch file.

If you need to use it manually, be sure to add the `ROS_NAMESPACE` if using `rosrun`.

For launch files, add the following line to your launch file:
```xml
<node pkg="detected_objects_visualizer" type="visualize_detected_objects" name="AN_INSTANCENAME_01"
          output="screen" ns="ROS_NAMESPACE"/>
```

i.e. to visualize the output of the `lidar tracker` use `ROS_NAMESPACE=/detection/lidar_tracker` in rosrun or
`ns="/detection/lidar_tracker"` in the launch file.