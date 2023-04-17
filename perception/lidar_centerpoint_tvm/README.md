# lidar_centerpoint_tvm

## Design

### Usage

lidar_centerpoint_tvm is a package for detecting dynamic 3D objects using TVM compiled centerpoint module for different backends. To use this package, replace `lidar_centerpoint` with `lidar_centerpoint_tvm` in perception launch files(for example, `lidar_based_detection.launch.xml` is lidar based detection is chosen.).

#### Neural network

This package will not build without a neural network for its inference.
The network is provided by the `tvm_utility` package.
See its design page for more information on how to enable downloading pre-compiled networks (by setting the `DOWNLOAD_ARTIFACTS` cmake variable), or how to handle user-compiled networks.

#### Backend

The backend used for the inference can be selected by setting the `lidar_centerpoint_tvm_BACKEND` cmake variable.
The current available options are `llvm` for a CPU backend, and `vulkan` or `opencl` for a GPU backend.
It defaults to `llvm`.

### Inputs / Outputs

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

| Name                            | Type   | Default Value | Description                                                 |
| ------------------------------- | ------ | ------------- | ----------------------------------------------------------- |
| `score_threshold`               | float  | `0.1`         | detected objects with score less than threshold are ignored |
| `densification_world_frame_id`  | string | `map`         | the world frame id to fuse multi-frame pointcloud           |
| `densification_num_past_frames` | int    | `1`           | the number of past frames to fuse with the current frame    |

### Bounding Box

The lidar segmentation node establishes a bounding box for the detected obstacles.
The `L-fit` method of fitting a bounding box to a cluster is used for that.

### Limitation and Known Issue

Due to an accuracy issue of `centerpoint` model, `vulkan` cannot be used at the moment.
As for 'llvm' backend, real-time performance cannot be achieved.

### Scatter Implementation

Scatter function can be implemented using either TVMScript or C++. For C++ implementation, please refer to <https://github.com/angry-crab/autoware.universe/blob/c020419fe52e359287eccb1b77e93bdc1a681e24/perception/lidar_centerpoint_tvm/lib/network/scatter.cpp#L65>

## Reference

[1] Yin, Tianwei, Xingyi Zhou, and Philipp Krähenbühl. "Center-based 3d object detection and tracking." arXiv preprint arXiv:2006.11275 (2020).

[2] Lang, Alex H., et al. "PointPillars: Fast encoders for object detection from point clouds." Proceedings of the IEEE/CVF Conference on Computer Vision and Pattern Recognition. 2019.

[3] <https://github.com/tianweiy/CenterPoint>

[4] <https://github.com/Abraham423/CenterPoint>

[5] <https://github.com/open-mmlab/OpenPCDet>

## Related issues

<!-- Required -->

- #908: Run Lidar Centerpoint with TVM
