# pointpainting_fusion

## Purpose

The `pointpainting_fusion` is a package for utilizing the class information detected by a 2D object detection in 3D object detection.

## Inner-workings / Algorithms

The lidar points are projected onto the output of an image-only 2d object detection network and the class scores are appended to each point. The painted point cloud can then be fed to the centerpoint network.

![pointpainting_fusion_image](./images/pointpainting_fusion.jpg)

## Inputs / Outputs

### Input

| Name                     | Type                                                     | Description                                               |
| ------------------------ | -------------------------------------------------------- | --------------------------------------------------------- |
| `input`                  | `sensor_msgs::msg::PointCloud2`                          | pointcloud                                                |
| `input/camera_info[0-7]` | `sensor_msgs::msg::CameraInfo`                           | camera information to project 3d points onto image planes |
| `input/rois[0-7]`        | `tier4_perception_msgs::msg::DetectedObjectsWithFeature` | ROIs from each image                                      |
| `input/image_raw[0-7]`   | `sensor_msgs::msg::Image`                                | images for visualization                                  |

### Output

| Name                     | Type                                             | Description              |
| ------------------------ | ------------------------------------------------ | ------------------------ |
| `output`                 | `sensor_msgs::msg::PointCloud2`                  | painted pointcloud       |
| `~/output/objects`       | `autoware_perception_msgs::msg::DetectedObjects` | detected objects         |
| `~/debug/image_raw[0-7]` | `sensor_msgs::msg::Image`                        | images for visualization |

## Parameters

### Core Parameters

| Name                                             | Type         | Default Value             | Description                                                 |
| ------------------------------------------------ | ------------ | ------------------------- | ----------------------------------------------------------- |
| `encoder_onnx_path`                              | string       | `""`                      | path to VoxelFeatureEncoder ONNX file                       |
| `encoder_engine_path`                            | string       | `""`                      | path to VoxelFeatureEncoder TensorRT Engine file            |
| `head_onnx_path`                                 | string       | `""`                      | path to DetectionHead ONNX file                             |
| `head_engine_path`                               | string       | `""`                      | path to DetectionHead TensorRT Engine file                  |
| `build_only`                                     | bool         | `false`                   | shutdown the node after TensorRT engine file is built       |
| `trt_precision`                                  | string       | `fp16`                    | TensorRT inference precision: `fp32` or `fp16`              |
| `post_process_params.score_threshold`            | double       | `0.4`                     | detected objects with score less than threshold are ignored |
| `post_process_params.yaw_norm_thresholds`        | list[double] | [0.3, 0.3, 0.3, 0.3, 0.0] | An array of distance threshold values of norm of yaw [rad]. |
| `post_process_params.iou_nms_target_class_names` | list[string] | ["CAR"]                   | An array of class names to be target in NMS.                |
| `post_process_params.iou_nms_search_distance_2d` | double       | 10.0                      | A maximum distance value to search the nearest objects.     |
| `post_process_params.iou_nms_threshold`          | double       | 0.1                       | A threshold value of NMS using IoU score.                   |
| `post_process_params.has_twist`                  | boolean      | false                     | Indicates whether the model outputs twist value.            |
| `densification_params.world_frame_id`            | string       | `map`                     | the world frame id to fuse multi-frame pointcloud           |
| `densification_params.num_past_frames`           | int          | `1`                       | the number of past frames to fuse with the current frame    |

## Assumptions / Known limits

- The multi-frame painting is not implemented yet.

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

## References/External links

<!-- cspell: ignore Vora, Sourabh -->

[1] Vora, Sourabh, et al. "PointPainting: Sequential fusion for 3d object detection." Proceedings of the IEEE/CVF Conference on Computer Vision and Pattern Recognition. 2020.

<!-- cspell: ignore Zhuangzhuang -->

[2] CVPR'20 Workshop on Scalability in Autonomous Driving] Waymo Open Dataset Challenge: <https://youtu.be/9g9GsI33ol8?t=535>
Ding, Zhuangzhuang, et al. "1st Place Solution for Waymo Open Dataset Challenge--3D Detection and Domain Adaptation." arXiv preprint arXiv:2006.15505 (2020).

## (Optional) Future extensions / Unimplemented parts

<!-- Write future extensions of this package.

Example:
  Currently, this package can't handle the chattering obstacles well. We plan to add some probabilistic filters in the perception layer to improve it.
  Also, there are some parameters that should be global(e.g. vehicle size, max steering, etc.). These will be refactored and defined as global parameters so that we can share the same parameters between different nodes.
-->
