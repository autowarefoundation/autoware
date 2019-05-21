# NDT SLAM
SLAM package using NDT scan matching.

### How to launch

* From a sourced terminal:
  - `roslaunch lidar_localizer ndt_slam.launch`


* From Runtime Manager:
Computing Tab -> lidar_localizer -> `ndt_slam`
  - You can change the config, as well as other parameters, by clicking [app]


### Subscribed topics

|Topic|Type|Description|
------|----|---------
|`/points_raw`|`sensor_msgs/PointCloud2`|PointCloud topic use for mapping.|
|`/filtered_points`|`sensor_msgs/PointCloud2`|PointCloud topic use for localizing.|
|`/points_map`|`sensor_msgs/PointCloud2`|PointCloud Map topic.|
|`/initialpose`|`geometry_msgs::PoseWithCovarianceStamped`| Initial position. Get from RViz's 2D Pose Estimate.|
|`/ekf_pose`|`geometry_msgs::PoseStamped`| Initial position. |
|`/config/ndt_slam`|`autoware_msgs/ConfigNDTSlam`|Configuration adjustment for threshold.|
|`/tf`|`tf2_msgs/TFMessage`|sensor frame <-> target frame, world frame <-> map frame|

### Published topics

|Topic|Type|Description|
------|----|---------
|`ndt_map`|`sensor_msgs/PointCloud2`| Mapping result. publish every 30 scans to speed up the process.|
|`ndt_pose`|`geometry_msgs/PoseStamped`| Estimated self position.|
|`ndt_pose_with_covariance`|`geometry_msgs/PoseWithCovarianceStamped`| Estimated self position with covariance. Covariance is calculated from the inverse of Hessian of NDT.|
|`predict_pose`|`geometry_msgs/PoseStamped`| Initial position before alignment.|
|`localizer_pose`|`geometry_msgs/PoseStamped`| Estimated sensor position.|
|`estimate_twist`|`geometry_msgs/TwistStamped`| Estimated Twist.|
|`matching_score`|`std_msgs/Float32`| Calculated from the nearest distance between map point cloud and LiDAR point cloud. Range of 0.0 to 1.0.|
|`matching_points`|`sensor_msgs/PointCloud2`|Comparing LiDAR PointCloud and PointCloud Map, topic of the PointCloud which matched.|
|`unmatching_points`|`sensor_msgs/PointCloud2`|Comparing LiDAR PointCloud and PointCloud Map, topic of the PointCloud which unmatched.|
|`time_ndt_matching`|`std_msgs/Float32`| processing time. use for debug.|

### Parameters
Launch file available parameters:

#### NDT Parameter

| Parameter | Type | Description | Default |
----------|-----|--------|-------
|`trans_epsilon`|*double*| The maximum difference between two consecutive transformations in order to consider convergence.| `0.01` |
|`step_size`|*double*| The newton line search maximum step length.| `0.1` |
|`resolution`|*double*| The ND voxel grid resolution.| `1.0` |
|`max_iterations`|*double*| The number of iterations required to calculate alignment.| `30.0` |

#### Mapping

| Parameter | Type | Description | Default |
----------|-----|--------|-------
|`with_mapping`|*bool*| If True, mapping simultaneously with position estimation.| `false` |
|`save_added_map`|*bool*| If True, save the map with only additional points. In addition, the map which merged the existing map pointcloud and the additional pointcloud is always saved. | `false` |
|`save_map_leaf_size`|*double*| The downsampling size of map.| `0.2` |
|`min_scan_range`|*double*| Minimum value of distance of LiDAR pointcloud used for mapping. It is used for the purpose of deleting extra things (such as the vehicle frame) around LiDAR.| `5.0` |
|`max_scan_range`|*double*| Maxmum value of distance of LiDAR pointcloud used for mapping. | `200.0` |
|`min_add_scan_shift`|*double*| Threshold that add pointcloud to the map if self-location moves more than this param.| `1.0` |
|`separate_mapping`|*bool*| If True, the map is divided and mapped to improve processing speed..| `false` |
|`separate_map_size`|*double*| Map split size.| `100.0` |
|`mapping_file_directory_path`|*string*| Map storage location.| `/tmp/Autoware/log/ndt_slam` |

#### Matching Score

| Parameter | Type | Description | Default |
----------|-----|--------|-------
|`matching_score_use_points_num`|*int*| Maximum number of pointcloud used for score calculation. If more than this value, it is downsampled by Ramdom sampling. As this value is larger, more accurate score calculation can be performed, but processing will be slower.| `300` |
|`matching_score_cutoff_lower_limit_z`|*double*| Use a point cloud less than this value for score calculation. This param is used to remove the point cloud of people and cars from score calculation.| `0.2` |
|`matching_score_cutoff_upper_limit_z`|*double*| Use a pointcloud higher than this value for score calculation. This param is used to remove the point cloud of people and cars from score calculation.| `2.0` |
|`matching_score_cutoff_lower_limit_range`|*double*| The minimum distance between pointcloud used for score calculation.| `5.0` |
|`matching_score_cutoff_upper_limit_range`|*double*| The maxmum distance between pointcloud used for score calculation.| `100.0` |
|`matching_score_kT`|*double*| kT of Fermi distribution function used for score calculation.| `0.05` |
|`matching_score_mu`|*double*| mu of Fermi distribution function used for score calculation.| `0.25` |

#### Coordinate system

| Parameter | Type | Description | Default |
----------|-----|--------|-------
|`target_frame`|*string*| Location of output results (ndt_pose, ndt_pose_with_covariance, predict_pose, estimate_twist ...).| `base_link` |
|`world_frame`|*string*|Coordinate system of output result (ndt_pose, ndt_pose_with_covariance, predict_pose, estimate_twist ...).| `map` |
|`publish_tf`|*bool*| If true, publish TF of map_frame <-> target_frame.| `true` |

#### Initial Pose

| Parameter | Type | Description | Default |
----------|-----|--------|-------
|`init_x`|*double*| Initial x position at initialization.| `0.0` |
|`init_y`|*double*| Initial y position at initialization.| `0.0` |
|`init_z`|*double*| Initial z position at initialization.| `0.0` |
|`init_roll`|*double*| Initial roll position at initialization.| `0.0` |
|`init_pitch`|*double*| Initial pitch position at initialization.| `0.0` |
|`init_yaw`|*double*| Initial yaw position at initialization.| `0.0` |

#### Others

| Parameter | Type | Description | Default |
----------|-----|--------|-------
|`points_queue_size`|*int*| Subscriber queue size of mapping_points_topic and localizing_points_topic. When offline mapping, it is recommended to use a large value (ex. 10000). | `1` |
|`use_fusion_localizer_feedback`|*bool*| If True, current_pose_topic is calculated as the initial position. It is mainly used when you want to feed back results such as EKF to NDT.| `false` |
|`use_nn_point_z_when_initial_pose`|*bool*| If True, the height value at the 2D Pose Estimate of RViz is complemented by the value of z at the nearest point of the map.| `false` |

### How to test

#### How to test mapping

  1. Download the rosbag. [Moriyama rosbag: http://db3.ertl.jp/autoware/sample_data/sample_moriyama_150324.tar.gz]

  1. Open run time manager.

  1. Simulation Tab, play rosbag and pause immediately.

  1. Setup Tab, launch `Baselink to Localizer TF`. (x, y, z, yaw, pitch, roll) = (1.2, 0.0, 2.0, 0.0, 0.0, 0.0).

  1. Sensing Tab -> Points Downsampler, launch `voxel_grid_filter`.

  1. Sensing Tab -> Computing -> lidar_localizer, open `ndt_slam` [app].
  Change `Points Queue Size` to 1000, check `With Mapping` and check `Separate Mapping`.

  1. Sensing Tab -> Computing -> lidar_localizer, launch `ndt_slam`.

  1. Simulation Tab, resume rosbag.

  1. Launch Rviz. Add Topic `ndt_map`(sensor_msgs/PointCloud2). And change `Fixed Frame` to `map`. You can see the state of mapping.

  1. To finish Mapping, kill the ndt_slam or uncheck `With Mapping` from [app]. Then save the map (By default it is stored in `/tmp/Autoware/log/ndt_slam/yyyymmdd_hhmmss/`).

  1. Check the map using pcl_viewer etc (pcl_viewer can install by `apt-get install pcl_tools`).

#### How to test localizing

  1. Open run time manager.

  1. Simulation Tab, play rosbag and pause immediately. Use the same rosbag as as Mapping.

  1. Setup Tab, launch `Baselink to Localizer TF`. (x, y, z, yaw, pitch, roll) = (1.2, 0.0, 2.0, 0.0, 0.0, 0.0).

  1. Map Tab, publish PointCloud Map to select file from `Ref` and press`Point Cloud`. Use the map created by `How to test mapping`.

  1. Map Tab, publish world to map TF to select file from `Ref` and press`TF`. TF file is `autoware/ros/src/.config/tf/tf_local.launch`.

  1. Sensing Tab -> Points Downsampler, launch `voxel_grid_filter`.

  1. Sensing Tab -> Computing -> lidar_localizer, open `ndt_slam` [app].
Change `Points Queue Size` to 1, uncheck `With Mapping`.

  1. Sensing Tab -> Computing -> lidar_localizer, launch `ndt_slam`.

  1. Simulation Tab, resume rosbag and pause after about one second.
  `ndt_slam` loads Map, then initializes KDTree, calculates covariance within each voxel, etc. It takes time to load a large map.
  After initialization, CPU resources of `ndt_slam` will be reduced.
  Please wait till then.

  1. Launch RViz. Add Topic `points_map`(sensor_msgs/PointCloud2) and `points_raw`(sensor_msgs/PointCloud2). And change `Fixed Frame` to `world`.

  1. Simulation Tab, resume rosbag.

  1. Set Initial pose with RViz `2D Pose Estimate` plugin.
  If you can set the initial position well, `points_map` and` points_raw` will be displayed overlapping.

  1. Add Topic `matching_score`(std_msgs/Float32) with RViz `Plotter2D`. You can see how much it matches the map. Evaluation value is 0.0 to 1.0, and it is higher as it is closer to 1.0.

### Video

#### How to Test Mapping
[![How to Test Mapping](http://img.youtube.com/vi/EEatuQnrxcg/hqdefault.jpg)](https://youtu.be/EEatuQnrxcg)

#### How to Test Localization
[![How to Test Localization](http://img.youtube.com/vi/dCyxSO3ZoyE/hqdefault.jpg)](https://youtu.be/dCyxSO3ZoyE)
