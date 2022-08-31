# blockage_diag

## Purpose

To ensure the performance of LiDAR and safety for autonomous driving, the abnormal condition diagnostics feature is needed.
LiDAR blockage is abnormal condition of LiDAR when some unwanted objects stitch to and block the light pulses and return signal.
This node's purpose is to detect the existing of blockage on LiDAR and its related size and location.

## Inner-workings / Algorithms

This node bases on the no-return region and its location to decide if it is a blockage.

![blockage situation](./image/blockage_diag.png)

The logic is showed as below

![blockage_diag_flowchart](./image/blockage_diag_flowchart.drawio.svg)

## Inputs / Outputs

This implementation inherits `pointcloud_preprocessor::Filter` class, please refer [README](../README.md).

### Input

| Name                        | Type                            | Description                                                     |
| --------------------------- | ------------------------------- | --------------------------------------------------------------- |
| `~/input/pointcloud_raw_ex` | `sensor_msgs::msg::PointCloud2` | The raw point cloud data is used to detect the no-return region |

### Output

| Name                                                 | Type                                    | Description                                        |
| ---------------------------------------------------- | --------------------------------------- | -------------------------------------------------- |
| `~/output/blockage_diag/debug/blockage_mask_image`   | `sensor_msgs::msg::Image`               | The mask image of detected blockage                |
| `~/output/blockage_diag/debug/ground_blockage_ratio` | `tier4_debug_msgs::msg::Float32Stamped` | The area ratio of blockage region in ground region |
| `~/output/blockage_diag/debug/sky_blockage_ratio`    | `tier4_debug_msgs::msg::Float32Stamped` | The area ratio of blockage region in sky region    |
| `~/output/blockage_diag/debug/lidar_depth_map`       | `sensor_msgs::msg::Image`               | The depth map image of input point cloud           |

## Parameters

| Name                       | Type   | Description                                        |
| -------------------------- | ------ | -------------------------------------------------- |
| `blockage_ratio_threshold` | float  | The threshold of blockage area ratio               |
| `blockage_count_threshold` | float  | The threshold of number continuous blockage frames |
| `horizontal_ring_id`       | int    | The id of horizontal ring of the LiDAR             |
| `angle_range`              | vector | The effective range of LiDAR                       |
| `vertical_bins`            | int    | The LiDAR channel number                           |
| `model`                    | string | The LiDAR model                                    |
| `buffering_frames`         | uint   | The number of buffering [range:1-200]              |
| `buffering_interval`       | uint   | The interval of buffering                          |

## Assumptions / Known limits

1. Only Hesai Pandar40P and Hesai PandarQT were tested. For a new LiDAR, it is neccessary to check order of channel id in vertical distribution manually and modifiy the code.
2. The current method is still limited for dust type of blockage when dust particles are sparsely distributed.

## (Optional) Error detection and handling

## (Optional) Performance characterization

## References/External links

## (Optional) Future extensions / Unimplemented parts
