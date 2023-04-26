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

#### Publish metadata of pointcloud map (ROS 2 topic)

The node publishes the pointcloud metadata attached with an ID. Metadata is loaded from the `.yaml` file. Please see [the description of `PointCloudMapMetaData.msg`](https://github.com/autowarefoundation/autoware_msgs/tree/main/autoware_map_msgs#pointcloudmapmetadatamsg) for details.

#### Send partial pointcloud map (ROS 2 service)

Here, we assume that the pointcloud maps are divided into grids.

Given a query from a client node, the node sends a set of pointcloud maps that overlaps with the queried area.
Please see [the description of `GetPartialPointCloudMap.srv`](https://github.com/autowarefoundation/autoware_msgs/tree/main/autoware_map_msgs#getpartialpointcloudmapsrv) for details.

#### Send differential pointcloud map (ROS 2 service)

Here, we assume that the pointcloud maps are divided into grids.

Given a query and set of map IDs, the node sends a set of pointcloud maps that overlap with the queried area and are not included in the set of map IDs.
Please see [the description of `GetDifferentialPointCloudMap.srv`](https://github.com/autowarefoundation/autoware_msgs/tree/main/autoware_map_msgs#getdifferentialpointcloudmapsrv) for details.

#### Send selected pointcloud map (ROS 2 service)

Here, we assume that the pointcloud maps are divided into grids.

Given IDs query from a client node, the node sends a set of pointcloud maps (each of which attached with unique ID) specified by query.
Please see [the description of `GetSelectedPointCloudMap.srv`](https://github.com/autowarefoundation/autoware_msgs/tree/main/autoware_map_msgs#getselectedpointcloudmapsrv) for details.

### Parameters

| Name                          | Type        | Description                                                                       | Default value |
| :---------------------------- | :---------- | :-------------------------------------------------------------------------------- | :------------ |
| enable_whole_load             | bool        | A flag to enable raw pointcloud map publishing                                    | true          |
| enable_downsampled_whole_load | bool        | A flag to enable downsampled pointcloud map publishing                            | false         |
| enable_partial_load           | bool        | A flag to enable partial pointcloud map server                                    | false         |
| enable_differential_load      | bool        | A flag to enable differential pointcloud map server                               | false         |
| enable_selected_load          | bool        | A flag to enable selected pointcloud map server                                   | false         |
| leaf_size                     | float       | Downsampling leaf size (only used when enable_downsampled_whole_load is set true) | 3.0           |
| pcd_paths_or_directory        | std::string | Path(s) to pointcloud map file or directory                                       |               |
| pcd_metadata_path             | std::string | Path to pointcloud metadata file                                                  |               |

### Interfaces

- `output/pointcloud_map` (sensor_msgs/msg/PointCloud2) : Raw pointcloud map
- `output/pointcloud_map_metadata` (autoware_map_msgs/msg/PointCloudMapMetaData) : Metadata of pointcloud map
- `output/debug/downsampled_pointcloud_map` (sensor_msgs/msg/PointCloud2) : Downsampled pointcloud map
- `service/get_partial_pcd_map` (autoware_map_msgs/srv/GetPartialPointCloudMap) : Partial pointcloud map
- `service/get_differential_pcd_map` (autoware_map_msgs/srv/GetDifferentialPointCloudMap) : Differential pointcloud map
- `service/get_selected_pcd_map` (autoware_map_msgs/srv/GetSelectedPointCloudMap) : Selected pointcloud map
- pointcloud map file(s) (.pcd)
- metadata of pointcloud map(s) (.yaml)

### Metadata

You must provide metadata in YAML format as well as pointcloud map files. Pointcloud map should be divided into one or more files with x-y grid.

Metadata should look like this:

```yaml
x_resolution: 100.0
y_resolution: 150.0
A.pcd: [1200, 2500] # -> 1200 < x < 1300, 2500 < y < 2650
B.pcd: [1300, 2500] # -> 1300 < x < 1400, 2500 < y < 2650
C.pcd: [1200, 2650] # -> 1200 < x < 1300, 2650 < y < 2800
D.pcd: [1400, 2650] # -> 1400 < x < 1500, 2650 < y < 2800
```

You may use [pointcloud_divider](https://github.com/MapIV/pointcloud_divider) from MAP IV for dividing pointcloud map as well as generating the compatible metadata.yaml.

### How to store map-related files

If you only have one pointcloud map, Autoware will assume the following directory structure by default.

```bash
sample-map-rosbag
├── lanelet2_map.osm
├── pointcloud_map.pcd
```

If you have multiple rosbags, an example directory structure would be as follows. Note that you need to have a metadata when you have multiple pointcloud map files.

```bash
sample-map-rosbag
├── lanelet2_map.osm
├── pointcloud_map
├── pcd_00.pcd
├── pcd_01.pcd
├── pcd_02.pcd
├── ...
└── pointcloud_map_metadata.yaml
```

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

lanelet2_map_visualization visualizes autoware_auto_mapping_msgs/HADMapBin messages into visualization_msgs/MarkerArray. There are 3 types of map can be loaded in autoware. Please makesure you selected the correct lanelet2_map_projector_type when you launch this package.

- MGRS
- UTM
- local

### How to Run

`ros2 run map_loader lanelet2_map_visualization`

### Subscribed Topics

- ~input/lanelet2_map (autoware_auto_mapping_msgs/HADMapBin) : binary data of Lanelet2 Map

### Published Topics

- ~output/lanelet2_map_marker (visualization_msgs/MarkerArray) : visualization messages for RViz

### Parameters

| Name                        | Type        | Description                                                  | Default value |
| :-------------------------- | :---------- | :----------------------------------------------------------- | :------------ |
| lanelet2_map_projector_type | std::string | The type of the map projector using, can be MGRS, UTM, local | MGRS          |
| latitude                    | double      | Latitude of map_origin, only using in UTM map projector      | 0.0           |
| longitude                   | double      | Longitude of map_origin, only using in UTM map projector     | 0.0           |
| center_line_resolution      | double      | Define the reolution of the lanelet center line              | 5.0           |
| lanelet2_map_path           | std::string | The lanelet2 map path                                        | None          |
