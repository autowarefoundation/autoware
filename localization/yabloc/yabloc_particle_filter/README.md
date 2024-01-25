# yabLoc_particle_filter

This package contains some executable nodes related to particle filter.

- [particle_predictor](#particle_predictor)
- [gnss_particle_corrector](#gnss_particle_corrector)
- [camera_particle_corrector](#camera_particle_corrector)

## particle_predictor

### Purpose

- This node performs predictive updating and resampling of particles.
- It retroactively reflects the particle weights determined by the corrector node.

### Inputs / Outputs

#### Input

| Name                          | Type                                             | Description                                               |
| ----------------------------- | ------------------------------------------------ | --------------------------------------------------------- |
| `input/initialpose`           | `geometry_msgs::msg::PoseWithCovarianceStamped`  | to specify the initial position of particles              |
| `input/twist_with_covariance` | `geometry_msgs::msg::TwistWithCovarianceStamped` | linear velocity and angular velocity of prediction update |
| `input/height`                | `std_msgs::msg::Float32`                         | ground height                                             |
| `input/weighted_particles`    | `yabloc_particle_filter::msg::ParticleArray`     | particles weighted by corrector nodes                     |

#### Output

| Name                           | Type                                            | Description                                               |
| ------------------------------ | ----------------------------------------------- | --------------------------------------------------------- |
| `output/pose_with_covariance`  | `geometry_msgs::msg::PoseWithCovarianceStamped` | particle centroid with covariance                         |
| `output/pose`                  | `geometry_msgs::msg::PoseStamped`               | particle centroid with covariance                         |
| `output/predicted_particles`   | `yabloc_particle_filter::msg::ParticleArray`    | particles weighted by predictor nodes                     |
| `debug/init_marker`            | `visualization_msgs::msg::Marker`               | debug visualization of initial position                   |
| `debug/particles_marker_array` | `visualization_msgs::msg::MarkerArray`          | particles visualization. published if `visualize` is true |

### Parameters

{{ json_to_markdown("localization/yabloc/yabloc_particle_filter/schema/predictor.schema.json") }}

### Services

| Name                 | Type                     | Description                                      |
| -------------------- | ------------------------ | ------------------------------------------------ |
| `yabloc_trigger_srv` | `std_srvs::srv::SetBool` | activation and deactivation of yabloc estimation |

## gnss_particle_corrector

### Purpose

- This node estimated particles weight using GNSS.
- It supports two types of input: `ublox_msgs::msg::NavPVT` and `geometry_msgs::msg::PoseWithCovarianceStamped`.

### Inputs / Outputs

#### Input

| Name                         | Type                                            | Description                                        |
| ---------------------------- | ----------------------------------------------- | -------------------------------------------------- |
| `input/height`               | `std_msgs::msg::Float32`                        | ground height                                      |
| `input/predicted_particles`  | `yabloc_particle_filter::msg::ParticleArray`    | predicted particles                                |
| `input/pose_with_covariance` | `geometry_msgs::msg::PoseWithCovarianceStamped` | gnss measurement. used if `use_ublox_msg` is false |
| `input/navpvt`               | `ublox_msgs::msg::NavPVT`                       | gnss measurement. used if `use_ublox_msg` is true  |

#### Output

| Name                           | Type                                         | Description                                               |
| ------------------------------ | -------------------------------------------- | --------------------------------------------------------- |
| `output/weighted_particles`    | `yabloc_particle_filter::msg::ParticleArray` | weighted particles                                        |
| `debug/gnss_range_marker`      | `visualization_msgs::msg::MarkerArray`       | gnss weight distribution                                  |
| `debug/particles_marker_array` | `visualization_msgs::msg::MarkerArray`       | particles visualization. published if `visualize` is true |

### Parameters

{{ json_to_markdown("localization/yabloc/yabloc_particle_filter/schema/gnss_particle_corrector.schema.json") }}

## camera_particle_corrector

### Purpose

- This node estimated particles weight using GNSS.

### Inputs / Outputs

#### Input

| Name                                  | Type                                         | Description                                                 |
| ------------------------------------- | -------------------------------------------- | ----------------------------------------------------------- |
| `input/predicted_particles`           | `yabloc_particle_filter::msg::ParticleArray` | predicted particles                                         |
| `input/ll2_bounding_box`              | `sensor_msgs::msg::PointCloud2`              | road surface markings converted to line segments            |
| `input/ll2_road_marking`              | `sensor_msgs::msg::PointCloud2`              | road surface markings converted to line segments            |
| `input/projected_line_segments_cloud` | `sensor_msgs::msg::PointCloud2`              | projected line segments                                     |
| `input/pose`                          | `geometry_msgs::msg::PoseStamped`            | reference to retrieve the area map around the self location |

#### Output

| Name                           | Type                                         | Description                                               |
| ------------------------------ | -------------------------------------------- | --------------------------------------------------------- |
| `output/weighted_particles`    | `yabloc_particle_filter::msg::ParticleArray` | weighted particles                                        |
| `debug/cost_map_image`         | `sensor_msgs::msg::Image`                    | cost map created from lanelet2                            |
| `debug/cost_map_range`         | `visualization_msgs::msg::MarkerArray`       | cost map boundary                                         |
| `debug/match_image`            | `sensor_msgs::msg::Image`                    | projected line segments image                             |
| `debug/scored_cloud`           | `sensor_msgs::msg::PointCloud2`              | weighted 3d line segments                                 |
| `debug/scored_post_cloud`      | `sensor_msgs::msg::PointCloud2`              | weighted 3d line segments which are iffy                  |
| `debug/state_string`           | `std_msgs::msg::String`                      | string describing the node state                          |
| `debug/particles_marker_array` | `visualization_msgs::msg::MarkerArray`       | particles visualization. published if `visualize` is true |

### Parameters

{{ json_to_markdown("localization/yabloc/yabloc_particle_filter/schema/camera_particle_corrector.schema.json") }}

### Services

| Name         | Type                     | Description                               |
| ------------ | ------------------------ | ----------------------------------------- |
| `switch_srv` | `std_srvs::srv::SetBool` | activation and deactivation of correction |
