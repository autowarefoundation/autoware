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

| Name                               | Type                                                  | Description              |
| ---------------------------------- | ----------------------------------------------------- | ------------------------ |
| `~/output/objects`                 | `autoware_auto_perception_msgs::msg::DetectedObjects` | detected objects         |
| `~/debug/pointcloud_densification` | `sensor_msgs::msg::PointCloud2`                       | densification pointcloud |

## Parameters

### Core Parameters

| Name                            | Type   | Default Value | Description                                                 |
| ------------------------------- | ------ | ------------- | ----------------------------------------------------------- |
| `score_threshold`               | float  | `0.4`         | detected objects with score less than threshold are ignored |
| `densification_world_frame_id`  | string | `map`         | the world frame id to fuse multi-frame pointcloud           |
| `densification_num_past_frames` | int    | `1`           | the number of past frames to fuse with the current frame    |
| `use_encoder_trt`               | bool   | `false`       | use TensorRT VoxelFeatureEncoder                            |
| `use_head_trt`                  | bool   | `true`        | use TensorRT DetectionHead                                  |
| `trt_precision`                 | string | `fp16`        | TensorRT inference precision: `fp32` or `fp16`              |
| `encoder_onnx_path`             | string | `""`          | path to VoxelFeatureEncoder ONNX file                       |
| `encoder_engine_path`           | string | `""`          | path to VoxelFeatureEncoder TensorRT Engine file            |
| `encoder_pt_path`               | string | `""`          | path to VoxelFeatureEncoder TorchScript file                |
| `head_onnx_path`                | string | `""`          | path to DetectionHead ONNX file                             |
| `head_engine_path`              | string | `""`          | path to DetectionHead TensorRT Engine file                  |
| `head_pt_path`                  | string | `""`          | path to DetectionHead TorchScript file                      |

## Assumptions / Known limits

- The `object.existence_probability` is stored the value of classification confidence of a DNN, not probability.

- If you have an error like `'GOMP_4.5' not found`, replace the OpenMP library in libtorch.

  ```bash
  sudo apt install libgomp1 -y
  sudo rm /usr/local/libtorch/lib/libgomp-75eea7e8.so.1
  sudo ln -s /usr/lib/x86_64-linux-gnu/libgomp.so.1 /usr/local/libtorch/lib/libgomp-75eea7e8.so.1
  ```

- if `use_encoder_trt` is set `use_encoder_trt`, more GPU memory is allocated.

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

[2] Lang, Alex H., et al. "Pointpillars: Fast encoders for object detection from point clouds." Proceedings of the IEEE/CVF Conference on Computer Vision and Pattern Recognition. 2019.

[3] <https://github.com/tianweiy/CenterPoint>

[4] <https://github.com/open-mmlab/mmdetection3d>

[5] <https://github.com/open-mmlab/OpenPCDet>

[6] <https://github.com/poodarchu/Det3D>

[7] <https://github.com/xingyizhou/CenterNet>

[8] <https://github.com/lzccccc/SMOKE>

[9] <https://github.com/yukkysaito/autoware_perception>

[10] <https://github.com/pytorch/pytorch>

## (Optional) Future extensions / Unimplemented parts

<!-- Write future extensions of this package.

Example:
  Currently, this package can't handle the chattering obstacles well. We plan to add some probabilistic filters in the perception layer to improve it.
  Also, there are some parameters that should be global(e.g. vehicle size, max steering, etc.). These will be refactored and defined as global parameters so that we can share the same parameters between different nodes.
-->
