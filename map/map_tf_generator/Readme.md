# map_tf_generator

## Purpose

This node broadcasts `viewer` frames for visualization of pointcloud map in Rviz.
The position of `viewer` frames is the geometric center of input pointclouds.

Note that there is no module to need `viewer` frames and this is used only for visualization.

## Inner-workings / Algorithms

## Inputs / Outputs

### Input

| Name                  | Type                            | Description                                                       |
| --------------------- | ------------------------------- | ----------------------------------------------------------------- |
| `/map/pointcloud_map` | `sensor_msgs::msg::PointCloud2` | Subscribe pointcloud map to calculate position of `viewer` frames |

### Output

| Name         | Type                     | Description               |
| ------------ | ------------------------ | ------------------------- |
| `/tf_static` | `tf2_msgs/msg/TFMessage` | Broadcast `viewer` frames |

## Parameters

### Node Parameters

None

### Core Parameters

| Name           | Type   | Default Value | Explanation                           |
| -------------- | ------ | ------------- | ------------------------------------- |
| `viewer_frame` | string | viewer        | Name of `viewer` frame                |
| `map_frame`    | string | map           | The parent frame name of viewer frame |

## Assumptions / Known limits

TBD.
