# autoware_lidar_transfusion

## Purpose

The `autoware_lidar_transfusion` package is used for 3D object detection based on lidar data (x, y, z, intensity).

## Inner-workings / Algorithms

The implementation bases on TransFusion [1] work. It uses TensorRT library for data process and network inference.

We trained the models using <https://github.com/open-mmlab/mmdetection3d>.

## Inputs / Outputs

### Input

| Name                 | Type                            | Description       |
| -------------------- | ------------------------------- | ----------------- |
| `~/input/pointcloud` | `sensor_msgs::msg::PointCloud2` | Input pointcloud. |

### Output

| Name                                   | Type                                             | Description                 |
| -------------------------------------- | ------------------------------------------------ | --------------------------- |
| `~/output/objects`                     | `autoware_perception_msgs::msg::DetectedObjects` | Detected objects.           |
| `debug/cyclic_time_ms`                 | `tier4_debug_msgs::msg::Float64Stamped`          | Cyclic time (ms).           |
| `debug/pipeline_latency_ms`            | `tier4_debug_msgs::msg::Float64Stamped`          | Pipeline latency time (ms). |
| `debug/processing_time/preprocess_ms`  | `tier4_debug_msgs::msg::Float64Stamped`          | Preprocess (ms).            |
| `debug/processing_time/inference_ms`   | `tier4_debug_msgs::msg::Float64Stamped`          | Inference time (ms).        |
| `debug/processing_time/postprocess_ms` | `tier4_debug_msgs::msg::Float64Stamped`          | Postprocess time (ms).      |
| `debug/processing_time/total_ms`       | `tier4_debug_msgs::msg::Float64Stamped`          | Total processing time (ms). |

## Parameters

### TransFusion node

{{ json_to_markdown("perception/autoware_lidar_transfusion/schema/transfusion.schema.dummy.json") }}

### TransFusion model

{{ json_to_markdown("perception/autoware_lidar_transfusion/schema/transfusion_ml_package.schema.json") }}

### Detection class remapper

{{ json_to_markdown("perception/autoware_lidar_transfusion/schema/detection_class_remapper.schema.json") }}

### The `build_only` option

The `autoware_lidar_transfusion` node has `build_only` option to build the TensorRT engine file from the ONNX file.
Although it is preferred to move all the ROS parameters in `.param.yaml` file in Autoware Universe, the `build_only` option is not moved to the `.param.yaml` file for now, because it may be used as a flag to execute the build as a pre-task. You can execute with the following command:

```bash
ros2 launch autoware_lidar_transfusion lidar_transfusion.launch.xml build_only:=true
```

### The `log_level` option

The default logging severity level for `autoware_lidar_transfusion` is `info`. For debugging purposes, the developer may decrease severity level using `log_level` parameter:

```bash
ros2 launch autoware_lidar_transfusion lidar_transfusion.launch.xml log_level:=debug
```

## Assumptions / Known limits

This library operates on raw cloud data (bytes). It is assumed that the input pointcloud message has following format:

```python
[
  sensor_msgs.msg.PointField(name='x', offset=0, datatype=7, count=1),
  sensor_msgs.msg.PointField(name='y', offset=4, datatype=7, count=1),
  sensor_msgs.msg.PointField(name='z', offset=8, datatype=7, count=1),
  sensor_msgs.msg.PointField(name='intensity', offset=12, datatype=2, count=1)
]
```

This input may consist of other fields as well - shown format is required minimum.
For debug purposes, you can validate your pointcloud topic using simple command:

```bash
ros2 topic echo <input_topic> --field fields
```

## Trained Models

You can download the onnx format of trained models by clicking on the links below.

- TransFusion: [transfusion.onnx](https://awf.ml.dev.web.auto/perception/models/transfusion/t4xx1_90m/v2/transfusion.onnx)

The model was trained in TIER IV's internal database (~11k lidar frames) for 50 epochs.

### Changelog

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

[1] Xuyang Bai, Zeyu Hu, Xinge Zhu, Qingqiu Huang, Yilun Chen, Hongbo Fu and Chiew-Lan Tai. "TransFusion: Robust LiDAR-Camera Fusion for 3D Object Detection with Transformers." arXiv preprint arXiv:2203.11496 (2022). <!-- cspell:disable-line -->

[2] <https://github.com/wep21/CUDA-TransFusion>

[3] <https://github.com/open-mmlab/mmdetection3d>

[4] <https://github.com/open-mmlab/OpenPCDet>

[5] <https://www.nuscenes.org/nuscenes>

## (Optional) Future extensions / Unimplemented parts

<!-- Write future extensions of this package.

Example:
  Currently, this package can't handle the chattering obstacles well. We plan to add some probabilistic filters in the perception layer to improve it.
  Also, there are some parameters that should be global(e.g. vehicle size, max steering, etc.). These will be refactored and defined as global parameters so that we can share the same parameters between different nodes.
-->
