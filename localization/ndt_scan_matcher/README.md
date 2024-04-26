# ndt_scan_matcher

## Purpose

ndt_scan_matcher is a package for position estimation using the NDT scan matching method.

There are two main functions in this package:

- estimate position by scan matching
- estimate initial position via the ROS service using the Monte Carlo method

One optional function is regularization. Please see the regularization chapter in the back for details. It is disabled by default.

## Inputs / Outputs

### Input

| Name                                | Type                                            | Description                           |
| ----------------------------------- | ----------------------------------------------- | ------------------------------------- |
| `ekf_pose_with_covariance`          | `geometry_msgs::msg::PoseWithCovarianceStamped` | initial pose                          |
| `points_raw`                        | `sensor_msgs::msg::PointCloud2`                 | sensor pointcloud                     |
| `sensing/gnss/pose_with_covariance` | `sensor_msgs::msg::PoseWithCovarianceStamped`   | base position for regularization term |

> `sensing/gnss/pose_with_covariance` is required only when regularization is enabled.

### Output

| Name                              | Type                                            | Description                                                                                                                              |
| --------------------------------- | ----------------------------------------------- | ---------------------------------------------------------------------------------------------------------------------------------------- |
| `ndt_pose`                        | `geometry_msgs::msg::PoseStamped`               | estimated pose                                                                                                                           |
| `ndt_pose_with_covariance`        | `geometry_msgs::msg::PoseWithCovarianceStamped` | estimated pose with covariance                                                                                                           |
| `/diagnostics`                    | `diagnostic_msgs::msg::DiagnosticArray`         | diagnostics                                                                                                                              |
| `points_aligned`                  | `sensor_msgs::msg::PointCloud2`                 | [debug topic] pointcloud aligned by scan matching                                                                                        |
| `points_aligned_no_ground`        | `sensor_msgs::msg::PointCloud2`                 | [debug topic] no ground pointcloud aligned by scan matching                                                                              |
| `initial_pose_with_covariance`    | `geometry_msgs::msg::PoseWithCovarianceStamped` | [debug topic] initial pose used in scan matching                                                                                         |
| `multi_ndt_pose`                  | `geometry_msgs::msg::PoseArray`                 | [debug topic] estimated poses from multiple initial poses in real-time covariance estimation                                             |
| `multi_initial_pose`              | `geometry_msgs::msg::PoseArray`                 | [debug topic] initial poses for real-time covariance estimation                                                                          |
| `exe_time_ms`                     | `tier4_debug_msgs::msg::Float32Stamped`         | [debug topic] execution time for scan matching [ms]                                                                                      |
| `transform_probability`           | `tier4_debug_msgs::msg::Float32Stamped`         | [debug topic] score of scan matching                                                                                                     |
| `no_ground_transform_probability` | `tier4_debug_msgs::msg::Float32Stamped`         | [debug topic] score of scan matching based on no ground LiDAR scan                                                                       |
| `iteration_num`                   | `tier4_debug_msgs::msg::Int32Stamped`           | [debug topic] number of scan matching iterations                                                                                         |
| `initial_to_result_relative_pose` | `geometry_msgs::msg::PoseStamped`               | [debug topic] relative pose between the initial point and the convergence point                                                          |
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

#### Frame

{{ json_to_markdown("localization/ndt_scan_matcher/schema/sub/frame.json") }}

#### Sensor Points

{{ json_to_markdown("localization/ndt_scan_matcher/schema/sub/sensor_points.json") }}

#### Ndt

{{ json_to_markdown("localization/ndt_scan_matcher/schema/sub/ndt.json") }}

#### Initial Pose Estimation

{{ json_to_markdown("localization/ndt_scan_matcher/schema/sub/initial_pose_estimation.json") }}

#### Validation

{{ json_to_markdown("localization/ndt_scan_matcher/schema/sub/validation.json") }}

#### Score Estimation

{{ json_to_markdown("localization/ndt_scan_matcher/schema/sub/score_estimation.json") }}

#### Covariance

{{ json_to_markdown("localization/ndt_scan_matcher/schema/sub/covariance.json") }}

## Regularization

### Abstract

This is a function that adds the regularization term to the NDT optimization problem as follows.

$$
\begin{align}
    \min_{\mathbf{R},\mathbf{t}}
    \mathrm{NDT}(\mathbf{R},\mathbf{t})
    +\mathrm{scale\ factor}\cdot \left|
        \mathbf{R}^\top
        (\mathbf{t_{base}-\mathbf{t}})
        \cdot
        \begin{pmatrix}
            1\\
            0\\
            0
        \end{pmatrix}
        \right|^2
\end{align}
$$

, where t_base is base position measured by GNSS or other means.
NDT(R,t) stands for the pure NDT cost function.
The regularization term shifts the optimal solution to the base position in the longitudinal direction of the vehicle.
Only errors along the **longitudinal direction** with respect to the base position are considered; errors along Z-axis and lateral-axis error are not considered.

Although the regularization term has rotation as a parameter, the gradient and hessian associated with it is not computed to stabilize the optimization.
Specifically, the gradients are computed as follows.

$$
\begin{align}
    &g_x=\nabla_x \mathrm{NDT}(\mathbf{R},\mathbf{t}) + 2 \mathrm{scale\ factor} \cos\theta_z\cdot e_{\mathrm{longitudinal}}
    \\
    &g_y=\nabla_y \mathrm{NDT}(\mathbf{R},\mathbf{t}) + 2 \mathrm{scale\ factor} \sin\theta_z\cdot e_{\mathrm{longitudinal}}
    \\
    &g_z=\nabla_z \mathrm{NDT}(\mathbf{R},\mathbf{t})
    \\
    &g_\mathbf{R}=\nabla_\mathbf{R} \mathrm{NDT}(\mathbf{R},\mathbf{t})
\end{align}
$$

Regularization is disabled by default.
If you wish to use it, please edit the following parameters to enable it.

#### Where is regularization available

This feature is effective on feature-less roads where GNSS is available, such as

- bridges
- highways
- farm roads

By remapping the base position topic to something other than GNSS, as described below, it can be valid outside of these.

#### Using other base position

Other than GNSS, you can give other global position topics obtained from magnetic markers, visual markers or etc. if they are available in your environment.
(Currently Autoware does not provide a node that gives such pose.)
To use your topic for regularization, you need to remap the `input_regularization_pose_topic` with your topic in `ndt_scan_matcher.launch.xml`.
By default, it is remapped with `/sensing/gnss/pose_with_covariance`.

#### Limitations

Since this function determines the base position by linear interpolation from the recently subscribed poses,
topics that are published at a low frequency relative to the driving speed cannot be used.
Inappropriate linear interpolation may result in bad optimization results.

When using GNSS for base location, the regularization can have negative effects in tunnels, indoors, and near skyscrapers.
This is because if the base position is far off from the true value, NDT scan matching may converge to inappropriate optimal position.

### Parameters

{{ json_to_markdown("localization/ndt_scan_matcher/schema/sub/ndt_regularization.json") }}

Regularization is disabled by default because GNSS is not always accurate enough to serve the appropriate base position in any scenes.

If the scale_factor is too large, the NDT will be drawn to the base position and scan matching may fail.
Conversely, if it is too small, the regularization benefit will be lost.

Note that setting scale_factor to 0 is equivalent to disabling regularization.

### Example

The following figures show tested maps.

- The left is a map with enough features that NDT can successfully localize.
- The right is a map with so few features that the NDT cannot localize well.

<img src="./media/bridge_map.jpg" alt="drawing" width="300"/> <img src="./media/bridge_map_less_feature.jpg" alt="drawing" width="300"/>

The following figures show the trajectories estimated on the feature-less map with standard NDT and regularization-enabled NDT, respectively.
The color of the trajectory indicates the error (meter) from the reference trajectory, which is computed with the feature-rich map.

- The left figure shows that the pure NDT causes a longitudinal error in the bridge and is not able to recover.
- The right figure shows that the regularization suppresses the longitudinal error.

<img src="./media/trajectory_without_regularization.png" alt="drawing" width="300"/> <img src="./media/trajectory_with_regularization.png" alt="drawing" width="300"/>

## Dynamic map loading

Autoware supports dynamic map loading feature for `ndt_scan_matcher`. Using this feature, NDT dynamically requests for the surrounding pointcloud map to `pointcloud_map_loader`, and then receive and preprocess the map in an online fashion.

Using the feature, `ndt_scan_matcher` can theoretically handle any large size maps in terms of memory usage. (Note that it is still possible that there exists a limitation due to other factors, e.g. floating-point error)

<img src="./media/differential_area_loading.gif" alt="drawing" width="400"/>

### Additional interfaces

#### Additional outputs

| Name                          | Type                            | Description                                       |
| ----------------------------- | ------------------------------- | ------------------------------------------------- |
| `debug/loaded_pointcloud_map` | `sensor_msgs::msg::PointCloud2` | pointcloud maps used for localization (for debug) |

#### Additional client

| Name                | Type                                                   | Description        |
| ------------------- | ------------------------------------------------------ | ------------------ |
| `client_map_loader` | `autoware_map_msgs::srv::GetDifferentialPointCloudMap` | map loading client |

### Parameters

{{ json_to_markdown("localization/ndt_scan_matcher/schema/sub/dynamic_map_loading.json") }}

### Notes for dynamic map loading

To use dynamic map loading feature for `ndt_scan_matcher`, you also need to split the PCD files into grids (recommended size: 20[m] x 20[m])

Note that the dynamic map loading may FAIL if the map is split into two or more large size map (e.g. 1000[m] x 1000[m]). Please provide either of

- one PCD map file
- multiple PCD map files divided into small size (~20[m])

Here is a split PCD map for `sample-map-rosbag` from Autoware tutorial: [`sample-map-rosbag_split.zip`](https://github.com/autowarefoundation/autoware.universe/files/10349104/sample-map-rosbag_split.zip)

|   PCD files    | How NDT loads map(s) |
| :------------: | :------------------: |
|  single file   |  at once (standard)  |
| multiple files |     dynamically      |

## Scan matching score based on no ground LiDAR scan

### Abstract

This is a function that uses no ground LiDAR scan to estimate the scan matching score. This score can reflect the current localization performance more accurately.
[related issue](https://github.com/autowarefoundation/autoware.universe/issues/2044).

### Parameters

{{ json_to_markdown("localization/ndt_scan_matcher/schema/sub/score_estimation_no_ground_points.json") }}

## 2D real-time covariance estimation

### Abstract

Calculate 2D covariance (xx, xy, yx, yy) in real time using the NDT convergence from multiple initial poses.
The arrangement of multiple initial poses is efficiently limited by the Hessian matrix of the NDT score function.
In this implementation, the number of initial positions is fixed to simplify the code.
The covariance can be seen as error ellipse from ndt_pose_with_covariance setting on rviz2.
[original paper](https://www.fujipress.jp/jrm/rb/robot003500020435/).

Note that this function may spoil healthy system behavior if it consumes much calculation resources.

### Parameters

initial_pose_offset_model is rotated around (x,y) = (0,0) in the direction of the first principal component of the Hessian matrix.
initial_pose_offset_model_x & initial_pose_offset_model_y must have the same number of elements.

{{ json_to_markdown("localization/ndt_scan_matcher/schema/sub/covariance_covariance_estimation.json") }}

## Diagnostics

### scan_matching_status

<img src="./media/diagnostic_scan_matching_status.png" alt="drawing" width="600"/>

| Name                                      | Description                                                                            | Transition condition to Warning                                                                                                                                                                                                                                                                                                                                          | Transition condition to Error    | Whether to reject the estimation result (affects `skipping_publish_num`) |
| ----------------------------------------- | -------------------------------------------------------------------------------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------ | -------------------------------- | ------------------------------------------------------------------------ |
| `topic_time_stamp`                        | the time stamp of input topic                                                          | none                                                                                                                                                                                                                                                                                                                                                                     | none                             | no                                                                       |
| `sensor_points_size`                      | the size of sensor points                                                              | the size is 0                                                                                                                                                                                                                                                                                                                                                            | none                             | yes                                                                      |
| `sensor_points_delay_time_sec`            | the delay time of sensor points                                                        | the time is **longer** than `validation.lidar_topic_timeout_sec`                                                                                                                                                                                                                                                                                                         | none                             | yes                                                                      |
| `is_succeed_transform_sensor_points`      | whether transform sensor points is succeed or not                                      | none                                                                                                                                                                                                                                                                                                                                                                     | failed                           | yes                                                                      |
| `sensor_points_max_distance`              | the max distance of sensor points                                                      | the max distance is **shorter** than `sensor_points.required_distance`                                                                                                                                                                                                                                                                                                   | none                             | yes                                                                      |
| `is_activated`                            | whether the node is in the "activate" state or not                                     | not "activate" state                                                                                                                                                                                                                                                                                                                                                     | none                             | yes                                                                      |
| `is_succeed_interpolate_initial_pose`     | whether the interpolate of initial pose is succeed or not                              | failed. <br> (1) the size of `initial_pose_buffer_` is **smaller** than 2. <br> (2) the timestamp difference between initial_pose and sensor pointcloud is **longer** than `validation.initial_pose_timeout_sec`. <br> (3) distance difference between two initial poses used for linear interpolation is **longer** than `validation.initial_pose_distance_tolerance_m` | none                             | yes                                                                      |
| `is_set_map_points`                       | whether the map points is set or not                                                   | not set                                                                                                                                                                                                                                                                                                                                                                  | none                             | yes                                                                      |
| `iteration_num`                           | the number of times calculate alignment                                                | the number of times is **larger** than `ndt.max_iterations`                                                                                                                                                                                                                                                                                                              | none                             | yes                                                                      |
| `local_optimal_solution_oscillation_num`  | the number of times the solution is judged to be oscillating                           | the number of times is **larger** than 10                                                                                                                                                                                                                                                                                                                                | none                             | yes                                                                      |
| `transform_probability`                   | the score of how well the map aligns with the sensor points                            | the score is **smaller** than`score_estimation.converged_param_transform_probability` (only in the case of `score_estimation.converged_param_type` is 0=TRANSFORM_PROBABILITY)                                                                                                                                                                                           | none                             | yes                                                                      |
| `nearest_voxel_transformation_likelihood` | the score of how well the map aligns with the sensor points                            | the score is **smaller** than `score_estimation.converged_param_nearest_voxel_transformation_likelihood` (only in the case of `score_estimation.converged_param_type` is 1=NEAREST_VOXEL_TRANSFORMATION_LIKELIHOOD)                                                                                                                                                      | none                             | yes                                                                      |
| `distance_initial_to_result`              | the distance between the position before convergence processing and the position after | the distance is **longer** than 3                                                                                                                                                                                                                                                                                                                                        | none                             | no                                                                       |
| `execution_time`                          | the time for convergence processing                                                    | the time is **longer** than `validation.critical_upper_bound_exe_time_ms`                                                                                                                                                                                                                                                                                                | none                             | no                                                                       |
| `skipping_publish_num`                    | the number of times rejected estimation results consecutively                          | none                                                                                                                                                                                                                                                                                                                                                                     | the number of times is 5 or more | -                                                                        |

※The `sensor_points_callback` shares the same callback group as the `trigger_node_service` and `ndt_align_service`. Consequently, if the initial pose estimation takes too long, this diagnostic may become stale.

### initial_pose_subscriber_status

<img src="./media/diagnostic_initial_pose_subscriber_status.png" alt="drawing" width="600"/>

| Name                   | Description                                                        | Transition condition to Warning | Transition condition to Error |
| ---------------------- | ------------------------------------------------------------------ | ------------------------------- | ----------------------------- |
| `topic_time_stamp`     | the time stamp of input topic                                      | none                            | none                          |
| `is_activated`         | whether the node is in the "activate" state or not                 | not "activate" state            | none                          |
| `is_expected_frame_id` | whether the input frame_id is the same as `frame.map_frame` or not | none                            | not the same                  |

### regularization_pose_subscriber_status

<img src="./media/diagnostic_regularization_pose_subscriber_status.png" alt="drawing" width="600"/>

| Name               | Description                   | Transition condition to Warning | Transition condition to Error |
| ------------------ | ----------------------------- | ------------------------------- | ----------------------------- |
| `topic_time_stamp` | the time stamp of input topic | none                            | none                          |

### trigger_node_service_status

<img src="./media/diagnostic_trigger_node_service_status.png" alt="drawing" width="600"/>

| Name                      | Description                                        | Transition condition to Warning | Transition condition to Error |
| ------------------------- | -------------------------------------------------- | ------------------------------- | ----------------------------- |
| `service_call_time_stamp` | the time stamp of service calling                  | none                            | none                          |
| `is_activated`            | whether the node is in the "activate" state or not | none                            | none                          |
| `is_succeed_service`      | whether the process of service is succeed or not   | none                            | none                          |

※
This diagnostic is only published when the service is called, so it becomes stale after the initial pose estimation is completed.

### ndt_align_service_status

<img src="./media/diagnostic_ndt_align_service_status.png" alt="drawing" width="600"/>

| Name                                | Description                                                                                                                                                                                                                                             | Transition condition to Warning | Transition condition to Error                               |
| ----------------------------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- | ------------------------------- | ----------------------------------------------------------- |
| `service_call_time_stamp`           | the time stamp of service calling                                                                                                                                                                                                                       | none                            | none                                                        |
| `is_succeed_transform_initial_pose` | whether transform initial pose is succeed or not                                                                                                                                                                                                        | none                            | failed                                                      |
| `is_need_rebuild`                   | whether it need to rebuild the map. If the map has not been loaded yet or if `distance_last_update_position_to_current_position encounters` is an Error state, it is considered necessary to reconstruct the map, and `is_need_rebuild` becomes `True`. | none                            | none                                                        |
| `maps_size_before`                  | the number of maps before update map                                                                                                                                                                                                                    | none                            | none                                                        |
| `is_succeed_call_pcd_loader`        | whether call pcd_loader service is succeed or not                                                                                                                                                                                                       | failed                          | none                                                        |
| `maps_to_add_size`                  | the number of maps to be added                                                                                                                                                                                                                          | none                            | none                                                        |
| `maps_to_remove_size`               | the number of maps to be removed                                                                                                                                                                                                                        | none                            | none                                                        |
| `map_update_execution_time`         | the time for map updating                                                                                                                                                                                                                               | none                            | none                                                        |
| `maps_size_after`                   | the number of maps after update map                                                                                                                                                                                                                     | none                            | none                                                        |
| `is_updated_map`                    | whether map is updated. If the map update couldn't be performed or there was no need to update the map, it becomes `False`                                                                                                                              | none                            | `is_updated_map` is `False` but `is_need_rebuild` is `True` |
| `is_set_map_points`                 | whether the map points is set or not                                                                                                                                                                                                                    | not set                         | none                                                        |
| `is_set_sensor_points`              | whether the sensor points is set or not                                                                                                                                                                                                                 | not set                         | none                                                        |
| `best_particle_score`               | the best score of particle                                                                                                                                                                                                                              | none                            | none                                                        |
| `is_succeed_service`                | whether the process of service is succeed or not                                                                                                                                                                                                        | failed                          | none                                                        |

※
This diagnostic is only published when the service is called, so it becomes stale after the initial pose estimation is completed.

### map_update_status

<img src="./media/diagnostic_map_update_status.png" alt="drawing" width="600"/>

| Name                                                | Description                                                                                                                                                                                                                                             | Transition condition to Warning | Transition condition to Error                                                                           |
| --------------------------------------------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- | ------------------------------- | ------------------------------------------------------------------------------------------------------- |
| `timer_callback_time_stamp`                         | the time stamp of timer_callback calling                                                                                                                                                                                                                | none                            | none                                                                                                    |
| `is_activated`                                      | whether the node is in the "activate" state or not                                                                                                                                                                                                      | not "activate" state            | none                                                                                                    |
| `is_set_last_update_position`                       | whether the `last_update_position` is set or not                                                                                                                                                                                                        | not set                         | none                                                                                                    |
| `distance_last_update_position_to_current_position` | the distance of `last_update_position` to current position                                                                                                                                                                                              | none                            | (the distance + `dynamic_map_loading.lidar_radius`) is **larger** than `dynamic_map_loading.map_radius` |
| `is_need_rebuild`                                   | whether it need to rebuild the map. If the map has not been loaded yet or if `distance_last_update_position_to_current_position encounters` is an Error state, it is considered necessary to reconstruct the map, and `is_need_rebuild` becomes `True`. | none                            | none                                                                                                    |
| `maps_size_before`                                  | the number of maps before update map                                                                                                                                                                                                                    | none                            | none                                                                                                    |
| `is_succeed_call_pcd_loader`                        | whether call pcd_loader service is succeed or not                                                                                                                                                                                                       | failed                          | none                                                                                                    |
| `maps_to_add_size`                                  | the number of maps to be added                                                                                                                                                                                                                          | none                            | none                                                                                                    |
| `maps_to_remove_size`                               | the number of maps to be removed                                                                                                                                                                                                                        | none                            | none                                                                                                    |
| `map_update_execution_time`                         | the time for map updating                                                                                                                                                                                                                               | none                            | none                                                                                                    |
| `maps_size_after`                                   | the number of maps after update map                                                                                                                                                                                                                     | none                            | none                                                                                                    |
| `is_updated_map`                                    | whether map is updated. If the map update couldn't be performed or there was no need to update the map, it becomes `False`                                                                                                                              | none                            | `is_updated_map` is `False` but `is_need_rebuild` is `True`                                             |
