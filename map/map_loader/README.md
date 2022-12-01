# map_loader package

This package provides the features of loading various maps.

## pointcloud_map_loader

### Feature

`pointcloud_map_loader` provides pointcloud maps to the other Autoware nodes in various configurations.
Currently, it supports the following two types:

- Publish raw pointcloud map
- Send partial pointcloud map loading via ROS 2 service

#### Publish raw pointcloud map (ROS 2 topic)

The node publishes the raw pointcloud map loaded from the `.pcd` file(s).

#### Send partial pointcloud map (ROS 2 service)

Here, we assume that the pointcloud maps are divided into grids.

Given a query from a client node, the node sends a set of pointcloud maps that overlaps with the queried area.
Please see [the description of `GetPartialPointCloudMap.srv`](https://github.com/autowarefoundation/autoware_msgs/tree/main/autoware_map_msgs#getpartialpointcloudmapsrv) for details.

### Parameters

| Name                | Type | Description                                    | Default value |
| :------------------ | :--- | :--------------------------------------------- | :------------ |
| enable_whole_load   | bool | A flag to enable raw pointcloud map publishing | true          |
| enable_partial_load | bool | A flag to enable partial pointcloud map server | true          |

### Interfaces

- `output/pointcloud_map` (sensor_msgs/msg/PointCloud2) : Raw pointcloud map
- `service/get_partial_pcd_map` (autoware_map_msgs/srv/GetPartialPointCloudMap) : Partial pointcloud map

---

## lanelet2_map_loader

### Feature

lanelet2_map_loader loads Lanelet2 file and publishes the map data as autoware_auto_mapping_msgs/HADMapBin message.
The node projects lan/lon coordinates into MGRS coordinates.

### How to run

`ros2 run map_loader lanelet2_map_loader --ros-args -p lanelet2_map_path:=path/to/map.osm`

### Published Topics

- ~output/lanelet2_map (autoware_auto_mapping_msgs/HADMapBin) : Binary data of loaded Lanelet2 Map

---

## lanelet2_map_visualization

### Feature

lanelet2_map_visualization visualizes autoware_auto_mapping_msgs/HADMapBin messages into visualization_msgs/MarkerArray.

### How to Run

`ros2 run map_loader lanelet2_map_visualization`

### Subscribed Topics

- ~input/lanelet2_map (autoware_auto_mapping_msgs/HADMapBin) : binary data of Lanelet2 Map

### Published Topics

- ~output/lanelet2_map_marker (visualization_msgs/MarkerArray) : visualization messages for RViz
