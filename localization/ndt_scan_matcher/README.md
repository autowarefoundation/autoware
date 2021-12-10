# ndt_scan_matcher

## Purpose

ndt_scan_matcher is a package for position estimation using the NDT scan matching method.

There are two main functions in this package:

- estimate position by scan matching
- estimate initial position via the ROS service using the Monte Carlo method

## Inputs / Outputs

### Input

| Name                       | Type                                            | Description       |
| -------------------------- | ----------------------------------------------- | ----------------- |
| `ekf_pose_with_covariance` | `geometry_msgs::msg::PoseWithCovarianceStamped` | initial pose      |
| `pointcloud_map`           | `sensor_msgs::msg::PointCloud2`                 | map pointcloud    |
| `points_raw`               | `sensor_msgs::msg::PointCloud2`                 | sensor pointcloud |

### Output

| Name                              | Type                                            | Description                                                                                                                              |
| --------------------------------- | ----------------------------------------------- | ---------------------------------------------------------------------------------------------------------------------------------------- |
| `ndt_pose`                        | `geometry_msgs::msg::PoseStamped`               | estimated pose                                                                                                                           |
| `ndt_pose_with_covariance`        | `geometry_msgs::msg::PoseWithCovarianceStamped` | estimated pose with covariance                                                                                                           |
| `/diagnostics`                    | `diagnostic_msgs::msg::DiagnosticArray`         | diagnostics                                                                                                                              |
| `points_aligned`                  | `sensor_msgs::msg::PointCloud2`                 | [debug topic] pointcloud aligned by scan matching                                                                                        |
| `initial_pose_with_covariance`    | `geometry_msgs::msg::PoseWithCovarianceStamped` | [debug topic] initial pose used in scan matching                                                                                         |
| `exe_time_ms`                     | `tier4_debug_msgs::msg::Float32Stamped`         | [debug topic] execution time for scan matching [ms]                                                                                      |
| `transform_probability`           | `tier4_debug_msgs::msg::Float32Stamped`         | [debug topic] score of scan matching                                                                                                     |
| `iteration_num`                   | `tier4_debug_msgs::msg::Int32Stamped`           | [debug topic] number of scan matching iterations                                                                                         |
| `initial_to_result_distance`      | `tier4_debug_msgs::msg::Float32Stamped`         | [debug topic] distance difference between the initial point and the convergence point [m]                                                |
| `initial_to_result_distance_old`  | `tier4_debug_msgs::msg::Float32Stamped`         | [debug topic] distance difference between the older of the two initial points used in linear interpolation and the convergence point [m] |
| `initial_to_result_distance_new`  | `tier4_debug_msgs::msg::Float32Stamped`         | [debug topic] distance difference between the newer of the two initial points used in linear interpolation and the convergence point [m] |
| `ndt_marker`                      | `visualization_msgs::msg::MarkerArray`          | [debug topic] markers for debugging                                                                                                      |
| `monte_carlo_initial_pose_marker` | `visualization_msgs::msg::MarkerArray`          | [debug topic] particles used in initial position estimation                                                                              |

### Service

| Name            | Type                                                         | Description                      |
| --------------- | ------------------------------------------------------------ | -------------------------------- |
| `ndt_align_srv` | `autoware_localization_srvs::srv::PoseWithCovarianceStamped` | service to estimate initial pose |

## Parameters

### Core Parameters

| Name                                    | Type   | Description                                                                                     |
| --------------------------------------- | ------ | ----------------------------------------------------------------------------------------------- |
| `base_frame`                            | string | Vehicle reference frame                                                                         |
| `input_sensor_points_queue_size`        | int    | Subscriber queue size                                                                           |
| `ndt_implement_type`                    | int    | NDT implementation type (0=PCL_GENERIC, 1=PCL_MODIFIED, 2=OMP)                                  |
| `trans_epsilon`                         | double | The maximum difference between two consecutive transformations in order to consider convergence |
| `step_size`                             | double | The newton line search maximum step length                                                      |
| `resolution`                            | double | The ND voxel grid resolution [m]                                                                |
| `max_iterations`                        | int    | The number of iterations required to calculate alignment                                        |
| `converged_param_transform_probability` | double | Threshold for deciding whether to trust the estimation result                                   |
| `omp_neighborhood_search_method`        | int    | neighborhood search method in OMP (0=KDTREE, 1=DIRECT26, 2=DIRECT7, 3=DIRECT1)                  |
| `omp_num_threads`                       | int    | Number of threads used for parallel computing                                                   |
