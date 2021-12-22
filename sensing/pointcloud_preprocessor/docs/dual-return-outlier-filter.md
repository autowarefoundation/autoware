# dual_return_outlier_filter

## Purpose

The purpose is to remove point cloud noise such as fog and rain and publish visibility as a diagnostic topic.

## Inner-workings / Algorithms

This node can remove rain and fog by considering the light reflected from the object in two stages according to the attenuation factor. The `dual_return_outlier_filter` is named because it removes noise using data that contains two types of return values separated by attenuation factor, as shown in the figure below.

![outlier_filter-return_type](./image/outlier_filter-return_type.drawio.svg)

Therefore, in order to use this node, the sensor driver must publish custom data including `return_type`. please refer to [PointXYZIRADT](https://github.com/tier4/AutowareArchitectureProposal.iv/blob/5d8dff0db51634f0c42d2a3e87ca423fbee84348/sensing/preprocessor/pointcloud/pointcloud_preprocessor/include/pointcloud_preprocessor/outlier_filter/dual_return_outlier_filter_nodelet.hpp#L86-L96) data structure.

Another feature of this node is that it publishes visibility as a diagnostic topic. With this function, for example, in heavy rain, the sensing module can notify that the processing performance has reached its limit, which can lead to ensuring the safety of the vehicle.

![outlier_filter-dual_return_overall](./image/outlier_filter-dual_return_overall.drawio.svg)

The figure below describe how the node works.
![outlier_filter-dual_return_detail](./image/outlier_filter-dual_return_detail.drawio.svg)

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
| `visibility_threshold`             | float  | When the percentage of white pixels in the binary histogram falls below this parameter the diagnostic status becomes WARN |

## Assumptions / Known limits

Not recommended for use as it is under development.
Input data must be `PointXYZIRADT` type data including `return_type`.

## (Optional) Error detection and handling

## (Optional) Performance characterization

## References/External links

## (Optional) Future extensions / Unimplemented parts
