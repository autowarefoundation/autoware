# map_file package
## points_map_filter
### feature
points_map_filter_node subscribe pointcloud maps and current pose, the node extract pointcloud near to the current pose.

#### subscribed topics
/points_map (sensor_msgs/PointCloud2)  : Raw pointcloud map. This topic usually comes from points_map_loader.  
/current_pose (geometry_msgs/PoseStamped) : Current pose of the car. This topic usually comes from pose_relay node.  

#### published topics
/points_map/filtered (sensor_msgs/PointCloud2) : Filtered pointcloud submap.  

#### parameters
load_grid_size (double) : grid size of submap.  
load_trigger_distance (double) : if the car moves load_trigger_distance(m), the map filter publish filtered submap. 

### how it works
map_filter_node relay /points_map topic until it recieves /current_pose topic.  
Then, the /current_pose topic recieved, the map_filter_node publish submap.

## demonstration
[![IMAGE ALT TEXT HERE](http://img.youtube.com/vi/LpKIuI5b4DU/0.jpg)](http://www.youtube.com/watch?v=LpKIuI5b4DU)