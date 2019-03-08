# Point Pillars for 3D Object Detection: ver. 1.0

Autoware package for Point Pillars.  [Referenced paper](https://arxiv.org/abs/1812.05784).

## Requirements

CUDA Toolkit v9.0 or v10.0

CUDNN: Tested with v7.3.1

TensorRT: Tested with 5.0.2 -> [How to install](https://docs.nvidia.com/deeplearning/sdk/tensorrt-install-guide/index.html#installing)

## How to setup

1. Install CUDA from this [website](https://developer.nvidia.com/cuda-downloads)

2. Install CUDNN

3. [Download](https://docs.nvidia.com/deeplearning/sdk/tensorrt-install-guide/index.html#downloading) the TensorRT local repo file that matches the Ubuntu version you are using.

4. Install TensorRT from the Debian local repo package.

```
$ sudo dpkg -i  
nv-tensorrt-repo-ubuntu1x04-cudax.x-trt5.x.x.x-ga-yyyymmdd_1-1_amd64.deb
$ sudo apt-key add /var/nv-tensorrt-repo-cudax.x-trt5.x.x.x-ga-yyyymmdd/7fa2af80.pub

$ sudo apt-get update
$ sudo apt-get install tensorrt
```

4. Download the pretrained file from [here](https://github.com/cirpue49/kitti_pretrained_pp).

```
$ git clone git@github.com:cirpue49/kitti_pretrained_point_pillars.git
```



## How to launch

* Launch file:
`roslaunch lidar_point_pillars lidar_point_pillars.launch pfe_onnx_file:=/PATH/TO/FILE.onnx rpn_onnx_file:=/PATH/TO/FILE.onnx input_topic:=/points_raw`

* You can launch it through the runtime manager in Computing tab, as well.

## API
```
/**
* @brief Call PointPillars for the inference.
* @param[in] in_points_array pointcloud array
* @param[in] in_num_points Number of points
* @param[out] out_detections Output bounding box from the network
* @details This is an interface for the algorithm.
*/
void doInference(float* in_points_array, int in_num_points, std::vector<float> out_detections);
```

## Parameters

|Parameter| Type| Description|Default|
----------|-----|--------|----|
|`input_topic`|*String*|Input topic Pointcloud. |`/points_raw`|
|`baselink_support`|*Bool*|Whether to use baselink to adjust parameters. |`True`|
|`reproduce_result_mode`|*Bool*|Whether to enable reproducible result mode at the cost of the runtime. |`False`|
|`score_threshold`|*Float*|Minimum score required to include the result [0,1]|0.5|
|`nms_overlap_threshold`|*Float*|Minimum IOU required to have when applying NMS [0,1]|0.5|
|`pfe_onnx_file`|*String* |Path to the PFE onnx file||
|`rpn_onnx_file`|*String* |Path to the RPN onnx file||

## Outputs

|Topic|Type|Description|
|---|---|---|
|`/detection/lidar_detector/objects`|`autoware_msgs/DetectedObjetArray`|Array of Detected Objects in Autoware format|

## Notes

* To display the results in Rviz `objects_visualizer` is required.
(Launch file launches automatically this node).

* Pretrained models are available [here](https://github.com/cirpue49/kitti_pretrained_pp), trained with the help of the KITTI dataset. For this reason, these are not suitable for commercial purposes. Derivative works are bound to the BY-NC-SA 3.0 License. (https://creativecommons.org/licenses/by-nc-sa/3.0/)
