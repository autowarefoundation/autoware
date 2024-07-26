# synchronized OGM fusion

> For simplicity, we use OGM as the meaning of the occupancy grid map.

This package is used to fuse the OGMs from synchronized sensors. Especially for the lidar.

Here shows the example OGM for the this synchronized OGM fusion.

| left lidar OGM                    | right lidar OGM                     | top lidar OGM                   |
| --------------------------------- | ----------------------------------- | ------------------------------- |
| ![left](image/left_lidar_ogm.png) | ![right](image/right_lidar_ogm.png) | ![top](image/top_lidar_ogm.png) |

OGM fusion with asynchronous sensor outputs is not suitable for this package. Asynchronous OGM fusion is under construction.

## Processing flow

The processing flow of this package is shown in the following figure.

![data_flow](image/synchronized_grid_map_fusion.drawio.svg)

- Single Frame Fusion
  - Single frame fusion means that the OGMs from synchronized sensors are fused in a certain time frame $t=t_n$.
- Multi Frame Fusion
  - In the multi frame fusion process, current fused single frame OGM in $t_n$ is fused with the previous fused single frame OGM in $t_{n-1}$.

## I/O

| Input topic name   | Type                                 | Description                                                                       |
| ------------------ | ------------------------------------ | --------------------------------------------------------------------------------- |
| `input_ogm_topics` | list of nav_msgs::msg::OccupancyGrid | List of input topics for Occupancy Grid Maps. This parameter is given in list, so |

| Output topic name             | Type                         | Description                                                                   |
| ----------------------------- | ---------------------------- | ----------------------------------------------------------------------------- |
| `~/output/occupancy_grid_map` | nav_msgs::msg::OccupancyGrid | Output topic name of the fused Occupancy Grid Map.                            |
| `~/debug/single_frame_map`    | nav_msgs::msg::OccupancyGrid | (debug topic) Output topic name of the single frame fused Occupancy Grid Map. |

## Parameters

Synchronized OGM fusion node parameters are shown in the following table. Main parameters to be considered in the fusion node is shown as bold.

| Ros param name              | Sample value         | Description                                                   |
| --------------------------- | -------------------- | ------------------------------------------------------------- |
| **input_ogm_topics**        | ["topic1", "topic2"] | List of input topics for Occupancy Grid Maps                  |
| **input_ogm_reliabilities** | [0.8, 0.2]           | Weights for the reliability of each input topic               |
| **fusion_method**           | "overwrite"          | Method of fusion ("overwrite", "log-odds", "dempster-shafer") |
| match_threshold_sec         | 0.01                 | Matching threshold in milliseconds                            |
| timeout_sec                 | 0.1                  | Timeout duration in seconds                                   |
| input_offset_sec            | [0.0, 0.0]           | Offset time in seconds for each input topic                   |
| map*frame*                  | "map"                | Frame name for the fused map                                  |
| base*link_frame*            | "base_link"          | Frame name for the base link                                  |
| grid*map_origin_frame*      | "base_link"          | Frame name for the origin of the grid map                     |
| fusion_map_length_x         | 100.0                | Length of the fused map along the X-axis                      |
| fusion_map_length_y         | 100.0                | Length of the fused map along the Y-axis                      |
| fusion_map_resolution       | 0.5                  | Resolution of the fused map                                   |

Since this node assumes that the OGMs from synchronized sensors are generated in the same time, we need to tune the `match_threshold_sec`, `timeout_sec` and `input_offset_sec` parameters to successfully fuse the OGMs.

## Fusion methods

For the single frame fusion, the following fusion methods are supported.

| Fusion Method in parameter | Description                                                                                                                                                                                                                                                                                               |
| -------------------------- | --------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `overwrite`                | The value of the cell in the fused OGM is overwritten by the value of the cell in the OGM with the highest priority. <br> We set priority as `Occupied` > `Free` > `Unknown`.                                                                                                                             |
| `log-odds`                 | The value of the cell in the fused OGM is calculated by the log-odds ratio method, which is known as a Bayesian fusion method. <br> The log-odds of a probability $p$ can be written as $l_p = \log(\frac{p}{1-p})$. <br> And the fused log-odds is calculated by the sum of log-odds. $l_f = \Sigma l_p$ |
| `dempster-shafer`          | The value of the cell in the fused OGM is calculated by the Dempster-Shafer theory[1]. This is also popular method to handle multiple evidences. This package applied conflict escape logic in [2] for the performance. See references for the algorithm details.                                         |

For the multi frame fusion, currently only supporting `log-odds` fusion method.

## How to use

### launch fusion node

The minimum node launch will be like the following.

```xml
<?xml version="1.0"?>
<launch>
<arg name="output_topic" default="~/output/occupancy_grid_map"/>
<arg name="fusion_node_param_path" default="$(find-pkg-share autoware_probabilistic_occupancy_grid_map)/config/synchronized_grid_map_fusion_node.param.yaml"/>

<node name="synchronized_grid_map_fusion_node" exec="synchronized_grid_map_fusion_node" pkg="autoware_probabilistic_occupancy_grid_map" output="screen">
  <remap from="~/output/occupancy_grid_map" to="$(var output_topic)"/>
  <param from="$(var fusion_node_param_path)"/>
</node>
</launch>
```

### (Optional) Generate OGMs in each sensor frame

You need to generate OGMs in each sensor frame before achieving grid map fusion.

`autoware_probabilistic_occupancy_grid_map` package supports to generate OGMs for the each from the point cloud data.

<details>
<summary> Example launch.xml (click to expand) </summary>

```xml
<include file="$(find-pkg-share tier4_perception_launch)/launch/occupancy_grid_map/probabilistic_occupancy_grid_map.launch.xml">
    <arg name="input/obstacle_pointcloud" value="/perception/obstacle_segmentation/single_frame/pointcloud"/>
    <arg name="input/raw_pointcloud" value="/sensing/lidar/right/outlier_filtered/pointcloud_synchronized"/>
    <arg name="output" value="/perception/occupancy_grid_map/right_lidar/map"/>
    <arg name="map_frame" value="base_link"/>
    <arg name="scan_origin" value="velodyne_right"/>
    <arg name="use_intra_process" value="true"/>
    <arg name="use_multithread" value="true"/>
    <arg name="use_pointcloud_container" value="$(var use_pointcloud_container)"/>
    <arg name="pointcloud_container_name" value="$(var pointcloud_container_name)"/>
    <arg name="method" value="pointcloud_based_occupancy_grid_map"/>
    <arg name="param_file" value="$(find-pkg-share autoware_probabilistic_occupancy_grid_map)/config/pointcloud_based_occupancy_grid_map_fusion.param.yaml"/>
</include>


The minimum parameter for the OGM generation in each frame is shown in the following table.

|Parameter|Description|
|--|--|
|`input/obstacle_pointcloud`| The input point cloud data for the OGM generation. This point cloud data should be the point cloud data which is segmented as the obstacle.|
|`input/raw_pointcloud`| The input point cloud data for the OGM generation. This point cloud data should be the point cloud data which is not segmented as the obstacle. |
|`output`| The output topic of the OGM. |
|`map_frame`| The tf frame for the OGM center origin. |
|`scan_origin`| The tf frame for the sensor origin. |
|`method`| The method for the OGM generation. Currently we support `pointcloud_based_occupancy_grid_map` and `laser_scan_based_occupancy_grid_map`. The pointcloud based method is recommended. |
|`param_file`| The parameter file for the OGM generation. See [example parameter file](config/pointcloud_based_occupancy_grid_map_for_fusion.param.yaml) |

```

</details>

<br>

We recommend to use same `map_frame`, size and resolutions for the OGMs from synchronized sensors.  
Also, remember to set `enable_single_frame_mode` and `filter_obstacle_pointcloud_by_raw_pointcloud` to `true` in the `autoware_probabilistic_occupancy_grid_map` package (you do not need to set these parameters if you use the above example config file).

<br>

### Run both OGM generation node and fusion node

We prepared the launch file to run both OGM generation node and fusion node in [`grid_map_fusion_with_synchronized_pointclouds.launch.py`](launch/grid_map_fusion_with_synchronized_pointclouds.launch.py)

You can include this launch file like the following.

```xml
<include file="$(find-pkg-share autoware_probabilistic_occupancy_grid_map)/launch/grid_map_fusion_with_synchronized_pointclouds.launch.py">
  <arg name="output" value="/perception/occupancy_grid_map/fusion/map"/>
  <arg name="use_intra_process" value="true"/>
  <arg name="use_multithread" value="true"/>
  <arg name="use_pointcloud_container" value="$(var use_pointcloud_container)"/>
  <arg name="pointcloud_container_name" value="$(var pointcloud_container_name)"/>
  <arg name="method" value="pointcloud_based_occupancy_grid_map"/>
  <arg name="fusion_config_file" value="$(var fusion_config_file)"/>
  <arg name="ogm_config_file" value="$(var ogm_config_file)"/>
</include>
```

The minimum parameter for the launch file is shown in the following table.

| Parameter            | Description                                                                                                                                                                          |
| -------------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------ |
| `output`             | The output topic of the finally fused OGM.                                                                                                                                           |
| `method`             | The method for the OGM generation. Currently we support `pointcloud_based_occupancy_grid_map` and `laser_scan_based_occupancy_grid_map`. The pointcloud based method is recommended. |
| `fusion_config_file` | The parameter file for the grid map fusion. See [example parameter file](config/grid_map_fusion.param.yaml)                                                                          |
| `ogm_config_file`    | The parameter file for the OGM generation. See [example parameter file](config/pointcloud_based_occupancy_grid_map_for_fusion.param.yaml)                                            |

## References

- [1] Dempster, A. P., Laird, N. M., & Rubin, D. B. (1977). Maximum likelihood from incomplete data via the EM algorithm. Journal of the Royal Statistical Society. Series B (Methodological), 39(1), 1-38.
- [2] <https://www.diva-portal.org/smash/get/diva2:852457/FULLTEXT01.pdf>
