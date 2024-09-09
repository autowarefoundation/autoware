# RANSAC Ground Filter

## Purpose

The purpose of this node is that remove the ground points from the input pointcloud.

## Inner-workings / Algorithms

Apply the input points to the plane, and set the points at a certain distance from the plane as points other than the ground. Normally, whn using this method, the input points is filtered so that it is almost flat before use. Since the drivable area is often flat, there are methods such as filtering by lane.

## Inputs / Outputs

This implementation inherits `autoware::pointcloud_preprocessor::Filter` class, please refer [README](../README.md).

## Parameters

### Node Parameters

This implementation inherits `autoware::pointcloud_preprocessor::Filter` class, please refer [README](../README.md).

#### Core Parameters

| Name                    | Type   | Description                                                     |
| ----------------------- | ------ | --------------------------------------------------------------- |
| `base_frame`            | string | base_link frame                                                 |
| `has_static_tf_only`    | bool   | Flag to listen TF only once                                     |
| `unit_axis`             | string | The axis which we need to search ground plane                   |
| `max_iterations`        | int    | The maximum number of iterations                                |
| `outlier_threshold`     | double | The distance threshold to the model [m]                         |
| `plane_slope_threshold` | double | The slope threshold to prevent mis-fitting [deg]                |
| `voxel_size_x`          | double | voxel size x [m]                                                |
| `voxel_size_y`          | double | voxel size y [m]                                                |
| `voxel_size_z`          | double | voxel size z [m]                                                |
| `height_threshold`      | double | The height threshold from ground plane for no ground points [m] |
| `debug`                 | bool   | whether to output debug information                             |

## Assumptions / Known limits

- This method can't handle slopes.
- The input points is filtered so that it is almost flat.

## (Optional) Error detection and handling

## (Optional) Performance characterization

## References/External links

<https://pcl.readthedocs.io/projects/tutorials/en/latest/planar_segmentation.html>

## (Optional) Future extensions / Unimplemented parts
