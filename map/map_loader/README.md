# map_loader package

This package provides the features of loading various maps.

## pointcloud_map_loader

### Feature

`pointcloud_map_loader` provides pointcloud maps to the other Autoware nodes in various configurations.
Currently, it supports the following two types:

- Publish raw pointcloud map
- Publish downsampled pointcloud map
- Send partial pointcloud map loading via ROS 2 service
- Send differential pointcloud map loading via ROS 2 service

#### Publish raw pointcloud map (ROS 2 topic)

The node publishes the raw pointcloud map loaded from the `.pcd` file(s).

#### Publish downsampled pointcloud map (ROS 2 topic)

The node publishes the downsampled pointcloud map loaded from the `.pcd` file(s). You can specify the downsample resolution by changing the `leaf_size` parameter.

#### Send partial pointcloud map (ROS 2 service)

Here, we assume that the pointcloud maps are divided into grids.

Given a query from a client node, the node sends a set of pointcloud maps that overlaps with the queried area.
Please see [the description of `GetPartialPointCloudMap.srv`](https://github.com/autowarefoundation/autoware_msgs/tree/main/autoware_map_msgs#getpartialpointcloudmapsrv) for details.

#### Send differential pointcloud map (ROS 2 service)

Here, we assume that the pointcloud maps are divided into grids.

Given a query and set of map IDs, the node sends a set of pointcloud maps that overlap with the queried area and are not included in the set of map IDs.
Please see [the description of `GetDifferentialPointCloudMap.srv`](https://github.com/autowarefoundation/autoware_msgs/tree/main/autoware_map_msgs#getdifferentialpointcloudmapsrv) for details.

### Parameters

| Name                          | Type  | Description                                                                       | Default value |
| :---------------------------- | :---- | :-------------------------------------------------------------------------------- | :------------ |
| enable_whole_load             | bool  | A flag to enable raw pointcloud map publishing                                    | true          |
| enable_downsampled_whole_load | bool  | A flag to enable downsampled pointcloud map publishing                            | false         |
| enable_partial_load           | bool  | A flag to enable partial pointcloud map server                                    | false         |
| enable_differential_load      | bool  | A flag to enable differential pointcloud map server                               | false         |
| leaf_size                     | float | Downsampling leaf size (only used when enable_downsampled_whole_load is set true) | 3.0           |

### Interfaces

- `output/pointcloud_map` (sensor_msgs/msg/PointCloud2) : Raw pointcloud map
- `output/debug/downsampled_pointcloud_map` (sensor_msgs/msg/PointCloud2) : Downsampled pointcloud map
- `service/get_partial_pcd_map` (autoware_map_msgs/srv/GetPartialPointCloudMap) : Partial pointcloud map
- `service/get_differential_pcd_map` (autoware_map_msgs/srv/GetDifferentialPointCloudMap) : Differential pointcloud map

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
