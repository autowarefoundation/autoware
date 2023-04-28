# lidar_centerpoint

## Purpose

lidar_centerpoint is a package for detecting dynamic 3D objects.

## Inner-workings / Algorithms

In this implementation, CenterPoint [1] uses a PointPillars-based [2] network to inference with TensorRT.

We trained the models using <https://github.com/open-mmlab/mmdetection3d>.

## Inputs / Outputs

### Input

| Name                 | Type                            | Description      |
| -------------------- | ------------------------------- | ---------------- |
| `~/input/pointcloud` | `sensor_msgs::msg::PointCloud2` | input pointcloud |

### Output

| Name                       | Type                                                  | Description          |
| -------------------------- | ----------------------------------------------------- | -------------------- |
| `~/output/objects`         | `autoware_auto_perception_msgs::msg::DetectedObjects` | detected objects     |
| `debug/cyclic_time_ms`     | `tier4_debug_msgs::msg::Float64Stamped`               | cyclic time (msg)    |
| `debug/processing_time_ms` | `tier4_debug_msgs::msg::Float64Stamped`               | processing time (ms) |

## Parameters

### Core Parameters

| Name                            | Type         | Default Value | Description                                                   |
| ------------------------------- | ------------ | ------------- | ------------------------------------------------------------- |
| `score_threshold`               | float        | `0.4`         | detected objects with score less than threshold are ignored   |
| `densification_world_frame_id`  | string       | `map`         | the world frame id to fuse multi-frame pointcloud             |
| `densification_num_past_frames` | int          | `1`           | the number of past frames to fuse with the current frame      |
| `trt_precision`                 | string       | `fp16`        | TensorRT inference precision: `fp32` or `fp16`                |
| `encoder_onnx_path`             | string       | `""`          | path to VoxelFeatureEncoder ONNX file                         |
| `encoder_engine_path`           | string       | `""`          | path to VoxelFeatureEncoder TensorRT Engine file              |
| `head_onnx_path`                | string       | `""`          | path to DetectionHead ONNX file                               |
| `head_engine_path`              | string       | `""`          | path to DetectionHead TensorRT Engine file                    |
| `nms_iou_target_class_names`    | list[string] | -             | target classes for IoU-based Non Maximum Suppression          |
| `nms_iou_search_distance_2d`    | double       | -             | If two objects are farther than the value, NMS isn't applied. |
| `nms_iou_threshold`             | double       | -             | IoU threshold for the IoU-based Non Maximum Suppression       |
| `build_only`                    | bool         | `false`       | shutdown the node after TensorRT engine file is built         |

## Assumptions / Known limits

- The `object.existence_probability` is stored the value of classification confidence of a DNN, not probability.

## Trained Models

You can download the onnx format of trained models by clicking on the links below.

- Centerpoint : [pts_voxel_encoder_centerpoint.onnx](https://awf.ml.dev.web.auto/perception/models/centerpoint/v2/pts_voxel_encoder_centerpoint.onnx), [pts_backbone_neck_head_centerpoint.onnx](https://awf.ml.dev.web.auto/perception/models/centerpoint/v2/pts_backbone_neck_head_centerpoint.onnx)
- Centerpoint tiny: [pts_voxel_encoder_centerpoint_tiny.onnx](https://awf.ml.dev.web.auto/perception/models/centerpoint/v2/pts_voxel_encoder_centerpoint_tiny.onnx), [pts_backbone_neck_head_centerpoint_tiny.onnx](https://awf.ml.dev.web.auto/perception/models/centerpoint/v2/pts_backbone_neck_head_centerpoint_tiny.onnx)

`Centerpoint` was trained in `nuScenes` (~28k lidar frames) [8] and TIER IV's internal database (~11k lidar frames) for 60 epochs.
`Centerpoint tiny` was trained in `Argoverse 2` (~110k lidar frames) [9] and TIER IV's internal database (~11k lidar frames) for 20 epochs.

## Standalone inference and visualization

In addition to its use as a standard ROS node, `lidar_centerpoint` can also be used to perform inferences in an isolated manner.
To do so, execute the following launcher, where `pcd_path` is the path of the pointcloud to be used for inference.

```bash
ros2 launch lidar_centerpoint single_inference_lidar_centerpoint.launch.xml pcd_path:=test_pointcloud.pcd detections_path:=test_detections.ply
```

`lidar_centerpoint` generates a `ply` file in the provided `detections_path`, which contains the detections as triangle meshes.
These detections can be visualized by most 3D tools, but we also integrate a visualization UI using `Open3D` which is launched alongside `lidar_centerpoint`.

### Changelog

#### v1 (2022/07/06)

| Name               | URLs                                                                                                     | Description                                                                                                                        |
| ------------------ | -------------------------------------------------------------------------------------------------------- | ---------------------------------------------------------------------------------------------------------------------------------- |
| `centerpoint`      | [pts_voxel_encoder][v1-encoder-centerpoint] <br> [pts_backbone_neck_head][v1-head-centerpoint]           | There is a single change due to the limitation in the implementation of this package. `num_filters=[32, 32]` of `PillarFeatureNet` |
| `centerpoint_tiny` | [pts_voxel_encoder][v1-encoder-centerpoint-tiny] <br> [pts_backbone_neck_head][v1-head-centerpoint-tiny] | The same model as `default` of `v0`.                                                                                               |

These changes are compared with [this configuration](https://github.com/tianweiy/CenterPoint/blob/v0.2/configs/waymo/pp/waymo_centerpoint_pp_two_pfn_stride1_3x.py).

#### v0 (2021/12/03)

| Name      | URLs                                                                                   | Description                                                                                                                                          |
| --------- | -------------------------------------------------------------------------------------- | ---------------------------------------------------------------------------------------------------------------------------------------------------- |
| `default` | [pts_voxel_encoder][v0-encoder-default] <br> [pts_backbone_neck_head][v0-head-default] | There are two changes from the original CenterPoint architecture. `num_filters=[32]` of `PillarFeatureNet` and `ds_layer_strides=[2, 2, 2]` of `RPN` |

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

[1] Yin, Tianwei, Xingyi Zhou, and Philipp Krähenbühl. "Center-based 3d object detection and tracking." arXiv preprint arXiv:2006.11275 (2020).

[2] Lang, Alex H., et al. "PointPillars: Fast encoders for object detection from point clouds." Proceedings of the IEEE/CVF Conference on Computer Vision and Pattern Recognition. 2019.

[3] <https://github.com/tianweiy/CenterPoint>

[4] <https://github.com/open-mmlab/mmdetection3d>

[5] <https://github.com/open-mmlab/OpenPCDet>

[6] <https://github.com/yukkysaito/autoware_perception>

[7] <https://github.com/NVIDIA-AI-IOT/CUDA-PointPillars>

[8] <https://www.nuscenes.org/nuscenes>

[9] <https://www.argoverse.org/av2.html>

## (Optional) Future extensions / Unimplemented parts

<!-- Write future extensions of this package.

Example:
  Currently, this package can't handle the chattering obstacles well. We plan to add some probabilistic filters in the perception layer to improve it.
  Also, there are some parameters that should be global(e.g. vehicle size, max steering, etc.). These will be refactored and defined as global parameters so that we can share the same parameters between different nodes.
-->

[v0-encoder-default]: https://awf.ml.dev.web.auto/perception/models/pts_voxel_encoder_default.onnx
[v0-head-default]: https://awf.ml.dev.web.auto/perception/models/pts_backbone_neck_head_default.onnx
[v1-encoder-centerpoint]: https://awf.ml.dev.web.auto/perception/models/centerpoint/v1/pts_voxel_encoder_centerpoint.onnx
[v1-head-centerpoint]: https://awf.ml.dev.web.auto/perception/models/centerpoint/v1/pts_backbone_neck_head_centerpoint.onnx
[v1-encoder-centerpoint-tiny]: https://awf.ml.dev.web.auto/perception/models/centerpoint/v1/pts_voxel_encoder_centerpoint_tiny.onnx
[v1-head-centerpoint-tiny]: https://awf.ml.dev.web.auto/perception/models/centerpoint/v1/pts_backbone_neck_head_centerpoint_tiny.onnx
