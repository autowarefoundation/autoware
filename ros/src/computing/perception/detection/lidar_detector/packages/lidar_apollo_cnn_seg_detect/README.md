# CNN LiDAR Baidu Object Segmenter

Autoware package based on Baidu's object segmenter.

## Pre requisites

Caffe distributable installed in your home (`~/caffe/distribute`).

```
$ cd
$ git clone https://github.com/BVLC/caffe
$ cd caffe
```
Follow instructions from [Installing Caffe from source](http://caffe.berkeleyvision.org/installation.html).

* **Use offical Make compilation procedure**. 
* Do not use thirdparty CMake setup.

Compile and create distributable:
```
$ make
$ make distribute
```

**Recompile Autoware to build the node.**

## How to launch

* From a sourced terminal:

Using rosrun:
`rosrun lidar_apollo_cnn_seg_detect lidar_apollo_cnn_seg_detect _network_definition_file:=/PATH/TO/FILE.prototxt _pretrained_model_file:=/PATH/TO/WEIGHTS.caffemodel _points_src:=/points_raw`

Using launch file:
`roslaunch lidar_apollo_cnn_seg_detect lidar_apollo_cnn_seg_detect.launch network_definition_file:=/PATH/TO/FILE.prototxt pretrained_model_file:=/PATH/TO/WEIGHTS.caffemodel points_src:=/points_raw`

* From Runtime Manager:

Computing Tab -> Detection/ lidar_detector -> `lidar_cnn_baidu_detect`. Configure parameters using the `[app]` button.

## Parameters

|Parameter| Type| Description|Default|
----------|-----|--------|----|
|`network_definition_file`|*String*|Path to the network definition file (prototxt)||
|`pretrained_model_file`|*String* |Path to the Pretrained model (weights)||
|`points_src`|*String*|Input topic Pointcloud. Default.|`/points_raw`|
|`score_threshold`|*Double*|Minimum score required as given by the network to include the result (0.-1.)|0.6|
|`use_gpu`|*Bool*|Whether ot not to use a GPU device|`true`|
|`gpu_device_id`|*Int*|GPU ID|`0`|

## Outputs

|Topic|Type|Description|
|---|---|---|
|`/detection/lidar_detector/points_cluster`|`sensor_msgs/PointCloud2`|Colored PointCloud of the resulting detected objects|
|`/detection/lidar_detector/objects`|`autoware_msgs/DetectedObjetArray`|Array of Detected Objects in Autoware format|

## Notes

* To display the results in Rviz `objects_visualizer` is required.
(Launch file launches automatically this node).

* Pre trained models can be downloaded from the Apollo project repository.