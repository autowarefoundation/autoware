# autoware_compare_map_segmentation

## Purpose

The `autoware_compare_map_segmentation` is a package that filters the ground points from the input pointcloud by using map info (e.g. pcd, elevation map or split map pointcloud from map_loader interface).

## Inner-workings / Algorithms

### Compare Elevation Map Filter

Compare the z of the input points with the value of elevation_map. The height difference is calculated by the binary integration of neighboring cells. Remove points whose height difference is below the `height_diff_thresh`.

<p align="center">
  <img src="./media/compare_elevation_map.png" width="1000">
</p>

### Distance Based Compare Map Filter

This filter compares the input pointcloud with the map pointcloud using the `nearestKSearch` function of `kdtree` and removes points that are close to the map point cloud. The map pointcloud can be loaded statically at once at the beginning or dynamically as the vehicle moves.

### Voxel Based Approximate Compare Map Filter

The filter loads the map point cloud, which can be loaded statically at the beginning or dynamically during vehicle movement, and creates a voxel grid of the map point cloud. The filter uses the getCentroidIndexAt function in combination with the getGridCoordinates function from the VoxelGrid class to find input points that are inside the voxel grid and removes them.

### Voxel Based Compare Map Filter

The filter loads the map pointcloud (static loading whole map at once at beginning or dynamic loading during vehicle moving) and utilizes VoxelGrid to downsample map pointcloud.

For each point of input pointcloud, the filter use `getCentroidIndexAt` combine with `getGridCoordinates` function from VoxelGrid class to check if the downsampled map point existing surrounding input points. Remove the input point which has downsampled map point in voxels containing or being close to the point.

### Voxel Distance based Compare Map Filter

This filter is a combination of the distance_based_compare_map_filter and voxel_based_approximate_compare_map_filter. The filter loads the map point cloud, which can be loaded statically at the beginning or dynamically during vehicle movement, and creates a voxel grid and a k-d tree of the map point cloud. The filter uses the getCentroidIndexAt function in combination with the getGridCoordinates function from the VoxelGrid class to find input points that are inside the voxel grid and removes them. For points that do not belong to any voxel grid, they are compared again with the map point cloud using the radiusSearch function of the k-d tree and are removed if they are close enough to the map.

## Inputs / Outputs

### Compare Elevation Map Filter

#### Input

| Name                    | Type                            | Description      |
| ----------------------- | ------------------------------- | ---------------- |
| `~/input/points`        | `sensor_msgs::msg::PointCloud2` | reference points |
| `~/input/elevation_map` | `grid_map::msg::GridMap`        | elevation map    |

#### Output

| Name              | Type                            | Description     |
| ----------------- | ------------------------------- | --------------- |
| `~/output/points` | `sensor_msgs::msg::PointCloud2` | filtered points |

#### Parameters

| Name                 | Type   | Description                                                                     | Default value |
| :------------------- | :----- | :------------------------------------------------------------------------------ | :------------ |
| `map_layer_name`     | string | elevation map layer name                                                        | elevation     |
| `map_frame`          | float  | frame_id of the map that is temporarily used before elevation_map is subscribed | map           |
| `height_diff_thresh` | float  | Remove points whose height difference is below this value [m]                   | 0.15          |

### Other Filters

#### Input

| Name                            | Type                            | Description                                            |
| ------------------------------- | ------------------------------- | ------------------------------------------------------ |
| `~/input/points`                | `sensor_msgs::msg::PointCloud2` | reference points                                       |
| `~/input/map`                   | `sensor_msgs::msg::PointCloud2` | map (in case static map loading)                       |
| `/localization/kinematic_state` | `nav_msgs::msg::Odometry`       | current ego-vehicle pose (in case dynamic map loading) |

#### Output

| Name              | Type                            | Description     |
| ----------------- | ------------------------------- | --------------- |
| `~/output/points` | `sensor_msgs::msg::PointCloud2` | filtered points |

#### Parameters

| Name                            | Type   | Description                                                                                                                             | Default value |
| :------------------------------ | :----- | :-------------------------------------------------------------------------------------------------------------------------------------- | :------------ |
| `use_dynamic_map_loading`       | bool   | map loading mode selection, `true` for dynamic map loading, `false` for static map loading, recommended for no-split map pointcloud     | true          |
| `distance_threshold`            | float  | Threshold distance to compare input points with map points [m]                                                                          | 0.5           |
| `map_update_distance_threshold` | float  | Threshold of vehicle movement distance when map update is necessary (in dynamic map loading) [m]                                        | 10.0          |
| `map_loader_radius`             | float  | Radius of map need to be loaded (in dynamic map loading) [m]                                                                            | 150.0         |
| `timer_interval_ms`             | int    | Timer interval to check if the map update is necessary (in dynamic map loading) [ms]                                                    | 100           |
| `publish_debug_pcd`             | bool   | Enable to publish voxelized updated map in `debug/downsampled_map/pointcloud` for debugging. It might cause additional computation cost | false         |
| `downsize_ratio_z_axis`         | double | Positive ratio to reduce voxel_leaf_size and neighbor point distance threshold in z axis                                                | 0.5           |

## Assumptions / Known limits

## (Optional) Error detection and handling

## (Optional) Performance characterization

## (Optional) References/External links

## (Optional) Future extensions / Unimplemented parts
