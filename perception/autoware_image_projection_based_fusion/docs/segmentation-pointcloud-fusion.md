# segmentation_pointcloud_fusion

## Purpose

The node `segmentation_pointcloud_fusion` is a package for filtering pointcloud that are belong to less interesting region which is defined by semantic or instance segmentation by 2D image segmentation model.

## Inner-workings / Algorithms

- The pointclouds are projected onto the semantic/instance segmentation mask image which is the output from 2D segmentation neural network.
- The pointclouds are belong to segment such as road, sidewalk, building, vegetation, sky ... are considered as less important points for autonomous driving and could be removed.

![segmentation_pointcloud_fusion_image](./images/segmentation_pointcloud_fusion.png)

## Inputs / Outputs

### Input

| Name                     | Type                            | Description                                                                            |
| ------------------------ | ------------------------------- | -------------------------------------------------------------------------------------- |
| `input`                  | `sensor_msgs::msg::PointCloud2` | input pointcloud                                                                       |
| `input/camera_info[0-7]` | `sensor_msgs::msg::CameraInfo`  | camera information to project 3d points onto image planes                              |
| `input/rois[0-7]`        | `sensor_msgs::msg::Image`       | A gray-scale image of semantic segmentation mask, the pixel value is semantic class id |
| `input/image_raw[0-7]`   | `sensor_msgs::msg::Image`       | images for visualization                                                               |

### Output

| Name     | Type                            | Description                |
| -------- | ------------------------------- | -------------------------- |
| `output` | `sensor_msgs::msg::PointCloud2` | output filtered pointcloud |

## Parameters

### Core Parameters

{{ json_to_markdown("perception/autoware_image_projection_based_fusion/schema/segmentation_pointcloud_fusion.schema.json") }}

## Assumptions / Known limits

<!-- Write assumptions and limitations of your implementation.

Example:
  This algorithm assumes obstacles are not moving, so if they rapidly move after the vehicle started to avoid them, it might collide with them.
  Also, this algorithm doesn't care about blind spots. In general, since too close obstacles aren't visible due to the sensing performance limit, please take enough margin to obstacles.
-->

## (Optional) Error detection and handling

<!-- Write how to detect errors and how to recover from them.

Example:
  This package can handle up to 20 obstacles. If more obstacles found, this node will give up and raise diagnostic errors.
-->

## (Optional) Performance characterization

<!-- Write performance information like complexity. If it wouldn't be the bottleneck, not necessary.

Example:

  ### Complexity

  This algorithm is O(N).

  ### Processing time

  ...
-->

## (Optional) References/External links

<!-- Write links you referred to when you implemented.

Example:
  [1] {link_to_a_thesis}
  [2] {link_to_an_issue}
-->

## (Optional) Future extensions / Unimplemented parts

<!-- Write future extensions of this package.

Example:
  Currently, this package can't handle the chattering obstacles well. We plan to add some probabilistic filters in the perception layer to improve it.
  Also, there are some parameters that should be global(e.g. vehicle size, max steering, etc.). These will be refactored and defined as global parameters so that we can share the same parameters between different nodes.
-->
