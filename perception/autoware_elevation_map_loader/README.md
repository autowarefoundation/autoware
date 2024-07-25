# autoware_elevation_map_loader

## Purpose

This package provides elevation map for autoware_compare_map_segmentation.

## Inner-workings / Algorithms

Generate elevation_map from subscribed pointcloud_map and vector_map and publish it.
Save the generated elevation_map locally and load it from next time.

The elevation value of each cell is the average value of z of the points of the lowest cluster.  
Cells with No elevation value can be inpainted using the values of neighboring cells.

<p align="center">
  <img src="./media/elevation_map.png" width="1500">
</p>

## Inputs / Outputs

### Input

| Name                            | Type                                            | Description                                |
| ------------------------------- | ----------------------------------------------- | ------------------------------------------ |
| `input/pointcloud_map`          | `sensor_msgs::msg::PointCloud2`                 | The point cloud map                        |
| `input/vector_map`              | `autoware_map_msgs::msg::LaneletMapBin`         | (Optional) The binary data of lanelet2 map |
| `input/pointcloud_map_metadata` | `autoware_map_msgs::msg::PointCloudMapMetaData` | (Optional) The metadata of point cloud map |

### Output

| Name                         | Type                            | Description                                                          |
| ---------------------------- | ------------------------------- | -------------------------------------------------------------------- |
| `output/elevation_map`       | `grid_map_msgs::msg::GridMap`   | The elevation map                                                    |
| `output/elevation_map_cloud` | `sensor_msgs::msg::PointCloud2` | (Optional) The point cloud generated from the value of elevation map |

### Service

| Name                           | Type                                               | Description                                                                                                                               |
| ------------------------------ | -------------------------------------------------- | ----------------------------------------------------------------------------------------------------------------------------------------- |
| `service/get_selected_pcd_map` | `autoware_map_msgs::srv::GetSelectedPointCloudMap` | (Optional) service to request point cloud map. If pointcloud_map_loader uses selected pointcloud map loading via ROS 2 service, use this. |

## Parameters

### Node parameters

| Name                              | Type        | Description                                                                                                                                                          | Default value |
| :-------------------------------- | :---------- | :------------------------------------------------------------------------------------------------------------------------------------------------------------------- | :------------ |
| map_layer_name                    | std::string | elevation_map layer name                                                                                                                                             | elevation     |
| param_file_path                   | std::string | GridMap parameters config                                                                                                                                            | path_default  |
| elevation_map_directory           | std::string | elevation_map file (bag2)                                                                                                                                            | path_default  |
| map_frame                         | std::string | map_frame when loading elevation_map file                                                                                                                            | map           |
| use_inpaint                       | bool        | Whether to inpaint empty cells                                                                                                                                       | true          |
| inpaint_radius                    | float       | Radius of a circular neighborhood of each point inpainted that is considered by the algorithm [m]                                                                    | 0.3           |
| use_elevation_map_cloud_publisher | bool        | Whether to publish `output/elevation_map_cloud`                                                                                                                      | false         |
| use_lane_filter                   | bool        | Whether to filter elevation_map with vector_map                                                                                                                      | false         |
| lane_margin                       | float       | Margin distance from the lane polygon of the area to be included in the inpainting mask [m]. Used only when use_lane_filter=True.                                    | 0.0           |
| use_sequential_load               | bool        | Whether to get point cloud map by service                                                                                                                            | false         |
| sequential_map_load_num           | int         | The number of point cloud maps to load at once (only used when use_sequential_load is set true). This should not be larger than number of all point cloud map cells. | 1             |

### GridMap parameters

The parameters are described on `config/elevation_map_parameters.yaml`.

#### General parameters

| Name                                           | Type | Description                                                                                                  | Default value |
| :--------------------------------------------- | :--- | :----------------------------------------------------------------------------------------------------------- | :------------ |
| pcl_grid_map_extraction/num_processing_threads | int  | Number of threads for processing grid map cells. Filtering of the raw input point cloud is not parallelized. | 12            |

#### Grid map parameters

See: <https://github.com/ANYbotics/grid_map/tree/ros2/grid_map_pcl>

Resulting grid map parameters.

| Name                                                     | Type  | Description                                                                                                                                                            | Default value |
| :------------------------------------------------------- | :---- | :--------------------------------------------------------------------------------------------------------------------------------------------------------------------- | :------------ |
| pcl_grid_map_extraction/grid_map/min_num_points_per_cell | int   | Minimum number of points in the point cloud that have to fall within any of the grid map cells. Otherwise the cell elevation will be set to NaN.                       | 3             |
| pcl_grid_map_extraction/grid_map/resolution              | float | Resolution of the grid map. Width and length are computed automatically.                                                                                               | 0.3           |
| pcl_grid_map_extraction/grid_map/height_type             | int   | The parameter that determine the elevation of a cell `0: Smallest value among the average values of each cluster`, `1: Mean value of the cluster with the most points` | 1             |
| pcl_grid_map_extraction/grid_map/height_thresh           | float | Height range from the smallest cluster (Only for height_type 1)                                                                                                        | 1.0           |

### Point Cloud Pre-processing Parameters

#### Rigid body transform parameters

Rigid body transform that is applied to the point cloud before computing elevation.

| Name                                                | Type  | Description                                                                                                             | Default value |
| :-------------------------------------------------- | :---- | :---------------------------------------------------------------------------------------------------------------------- | :------------ |
| pcl_grid_map_extraction/cloud_transform/translation | float | Translation (xyz) that is applied to the input point cloud before computing elevation.                                  | 0.0           |
| pcl_grid_map_extraction/cloud_transform/rotation    | float | Rotation (intrinsic rotation, convention X-Y'-Z'') that is applied to the input point cloud before computing elevation. | 0.0           |

#### Cluster extraction parameters

Cluster extraction is based on pcl algorithms. See <https://pointclouds.org/documentation/tutorials/cluster_extraction.html> for more details.

| Name                                                         | Type  | Description                                                                            | Default value |
| :----------------------------------------------------------- | :---- | :------------------------------------------------------------------------------------- | :------------ |
| pcl_grid_map_extraction/cluster_extraction/cluster_tolerance | float | Distance between points below which they will still be considered part of one cluster. | 0.2           |
| pcl_grid_map_extraction/cluster_extraction/min_num_points    | int   | Min number of points that a cluster needs to have (otherwise it will be discarded).    | 3             |
| pcl_grid_map_extraction/cluster_extraction/max_num_points    | int   | Max number of points that a cluster can have (otherwise it will be discarded).         | 1000000       |

#### Outlier removal parameters

See <https://pointclouds.org/documentation/tutorials/statistical_outlier.html> for more explanation on outlier removal.

| Name                                                       | Type  | Description                                                                    | Default value |
| :--------------------------------------------------------- | :---- | :----------------------------------------------------------------------------- | :------------ |
| pcl_grid_map_extraction/outlier_removal/is_remove_outliers | float | Whether to perform statistical outlier removal.                                | false         |
| pcl_grid_map_extraction/outlier_removal/mean_K             | float | Number of neighbors to analyze for estimating statistics of a point.           | 10            |
| pcl_grid_map_extraction/outlier_removal/stddev_threshold   | float | Number of standard deviations under which points are considered to be inliers. | 1.0           |

#### Subsampling parameters

See <https://pointclouds.org/documentation/tutorials/voxel_grid.html> for more explanation on point cloud downsampling.

| Name                                                     | Type  | Description                             | Default value |
| :------------------------------------------------------- | :---- | :-------------------------------------- | :------------ |
| pcl_grid_map_extraction/downsampling/is_downsample_cloud | bool  | Whether to perform downsampling or not. | false         |
| pcl_grid_map_extraction/downsampling/voxel_size          | float | Voxel sizes (xyz) in meters.            | 0.02          |
