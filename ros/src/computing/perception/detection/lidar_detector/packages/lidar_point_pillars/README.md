# Point Pillars for 3D Object Detection

Autoware package for Point Pillars.  ([reference paper](https://arxiv.org/abs/1812.05784))

## Pre requisites

CUDA Toolkit v9.0 or 10.0

TensorRT : [How to install](https://docs.nvidia.com/deeplearning/sdk/tensorrt-install-guide/index.html#installing)


## How to launch

Using launch file:
`roslaunch lidar_point_pillars lidar_point_pillars.launch pfe_onnx_file:=/PATH/TO/FILE.onnx rpn_onnx_file:=/PATH/TO/FILE.onnx input_topic:=/points_raw`


## Parameters

|Parameter| Type| Description|Default|
----------|-----|--------|----|
|`input_topic`|*String*|Input topic Pointcloud. Default.|`/points_raw`|
|`baselink_support`|*Bool*|Whether to use baselink to adjust parameters. Default.|`True`|
|`reproduce_result_mode`|*Bool*|Whether to enable reproducible result mode at the cost of the runtime. Default.|`False`|
|`score_threshold`|*Float*|Minimum score required to include the result (0.1)|0.5|
|`nms_overlap_threshold`|*Float*|Minimum IOU required to have when applying NMS (0.1)|0.5|
|`pfe_onnx_file`|*String* |Path to the PFE onnx file||
|`rpn_onnx_file`|*String* |Path to the RPN onnx file||

## Outputs

|Topic|Type|Description|
|---|---|---|
|`/detection/lidar_detector/objects`|`autoware_msgs/DetectedObjetArray`|Array of Detected Objects in Autoware format|

## Notes

* To display the results in Rviz `objects_visualizer` is required.
(Launch file launches automatically this node).

* Pre trained models can be downloaded from the [github repository](https://github.com/cirpue49/kitti_pretrained_pp).Notice that this model is under `BY-NC-SA 3.0` license.

* If trained model comes from KITTI data, users might not be allowed to use the model for Commercial purpose.
