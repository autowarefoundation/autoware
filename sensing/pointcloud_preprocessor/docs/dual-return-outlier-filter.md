# dual_return_outlier_filter

## Purpose

The purpose is to remove point cloud noise such as fog and rain and publish visibility as a diagnostic topic.

## Inner-workings / Algorithms

This node can remove rain and fog by considering the light reflected from the object in two stages according to the attenuation factor. The `dual_return_outlier_filter` is named because it removes noise using data that contains two types of return values separated by attenuation factor, as shown in the figure below.

![outlier_filter-return_type](./image/outlier_filter-return_type.drawio.svg)

Therefore, in order to use this node, the sensor driver must publish custom data including `return_type`. please refer to [PointXYZIRADRT](../../../common/autoware_point_types/include/autoware_point_types/types.hpp#L57-L76) data structure.

Another feature of this node is that it publishes visibility as a diagnostic topic. With this function, for example, in heavy rain, the sensing module can notify that the processing performance has reached its limit, which can lead to ensuring the safety of the vehicle.

In some complicated road scenes where normal objects also reflect the light in two stages, for instance plants, leaves, some plastic net etc, the visibility faces some drop in fine weather condition. To deal with that, optional settings of a region of interest (ROI) are added.

1. `Fixed_xyz_ROI` mode: Visibility estimation based on the weak points in a fixed cuboid surrounding region of ego-vehicle, defined by x, y, z in base_link perspective.
2. `Fixed_azimuth_ROI` mode: Visibility estimation based on the weak points in a fixed surrounding region of ego-vehicle, defined by azimuth and distance of LiDAR perspective.

When select 2 fixed ROI modes, due to the range of weak points is shrink, the sensitivity of visibility is decrease so that a trade of between `weak_first_local_noise_threshold` and `visibility_threshold` is needed.

![outlier_filter-dual_return_overall](./image/outlier_filter-dual_return_overall.drawio.svg)

The figure below describe how the node works.
![outlier_filter-dual_return_detail](./image/outlier_filter-dual_return_detail.drawio.svg)

The below picture shows the ROI options.

![outlier_filter-dual_return_ROI_setting_options](./image/outlier_filter-dual_return_ROI_setting_options.png)

## Inputs / Outputs

This implementation inherits `pointcloud_preprocessor::Filter` class, please refer [README](../README.md).

### Output

| Name                                           | Type                                    | Description                                             |
| ---------------------------------------------- | --------------------------------------- | ------------------------------------------------------- |
| `/dual_return_outlier_filter/frequency_image`  | `sensor_msgs::msg::Image`               | The histogram image that represent visibility           |
| `/dual_return_outlier_filter/visibility`       | `tier4_debug_msgs::msg::Float32Stamped` | A representation of visibility with a value from 0 to 1 |
| `/dual_return_outlier_filter/pointcloud_noise` | `sensor_msgs::msg::Pointcloud2`         | The pointcloud removed as noise                         |

## Parameters

### Node Parameters

This implementation inherits `pointcloud_preprocessor::Filter` class, please refer [README](../README.md).

### Core Parameters

| Name                               | Type   | Description                                                                                                               |
| ---------------------------------- | ------ | ------------------------------------------------------------------------------------------------------------------------- |
| `vertical_bins`                    | int    | The number of vertical bin for visibility histogram                                                                       |
| `max_azimuth_diff`                 | float  | Threshold for ring_outlier_filter                                                                                         |
| `weak_first_distance_ratio`        | double | Threshold for ring_outlier_filter                                                                                         |
| `general_distance_ratio`           | double | Threshold for ring_outlier_filter                                                                                         |
| `weak_first_local_noise_threshold` | int    | The parameter for determining whether it is noise                                                                         |
| `visibility_error_threshold`       | float  | When the percentage of white pixels in the binary histogram falls below this parameter the diagnostic status becomes ERR  |
| `visibility_warn_threshold`        | float  | When the percentage of white pixels in the binary histogram falls below this parameter the diagnostic status becomes WARN |
| `roi_mode`                         | string | The name of ROI mode for switching                                                                                        |
| `min_azimuth_deg`                  | float  | The left limit of azimuth for `Fixed_azimuth_ROI` mode                                                                    |
| `max_azimuth_deg`                  | float  | The right limit of azimuth for `Fixed_azimuth_ROI` mode                                                                   |
| `max_distance`                     | float  | The limit distance for for `Fixed_azimuth_ROI` mode                                                                       |
| `x_max`                            | float  | Maximum of x for `Fixed_xyz_ROI` mode                                                                                     |
| `x_min`                            | float  | Minimum of x for `Fixed_xyz_ROI` mode                                                                                     |
| `y_max`                            | float  | Maximum of y for `Fixed_xyz_ROI` mode                                                                                     |
| `y_min`                            | float  | Minimum of y for `Fixed_xyz_ROI` mode                                                                                     |
| `z_max`                            | float  | Maximum of z for `Fixed_xyz_ROI` mode                                                                                     |
| `z_min`                            | float  | Minimum of z for `Fixed_xyz_ROI` mode                                                                                     |

## Assumptions / Known limits

Not recommended for use as it is under development.
Input data must be `PointXYZIRADRT` type data including `return_type`.

## (Optional) Error detection and handling

## (Optional) Performance characterization

## References/External links

## (Optional) Future extensions / Unimplemented parts
