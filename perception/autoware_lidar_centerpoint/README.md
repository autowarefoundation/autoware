# autoware_lidar_centerpoint

## Purpose

autoware_lidar_centerpoint is a package for detecting dynamic 3D objects.

## Inner-workings / Algorithms

In this implementation, CenterPoint [1] uses a PointPillars-based [2] network to inference with TensorRT.

We trained the models using <https://github.com/open-mmlab/mmdetection3d>.

## Inputs / Outputs

### Input

| Name                 | Type                            | Description      |
| -------------------- | ------------------------------- | ---------------- |
| `~/input/pointcloud` | `sensor_msgs::msg::PointCloud2` | input pointcloud |

### Output

| Name                       | Type                                             | Description          |
| -------------------------- | ------------------------------------------------ | -------------------- |
| `~/output/objects`         | `autoware_perception_msgs::msg::DetectedObjects` | detected objects     |
| `debug/cyclic_time_ms`     | `tier4_debug_msgs::msg::Float64Stamped`          | cyclic time (msg)    |
| `debug/processing_time_ms` | `tier4_debug_msgs::msg::Float64Stamped`          | processing time (ms) |

## Parameters

### ML Model Parameters

Note that these parameters are associated with ONNX file, predefined during the training phase. Be careful to change ONNX file as well when changing this parameter. Also, whenever you update the ONNX file, do NOT forget to check these values.

| Name                                   | Type         | Default Value                                    | Description                                                           |
| -------------------------------------- | ------------ | ------------------------------------------------ | --------------------------------------------------------------------- |
| `model_params.class_names`             | list[string] | ["CAR", "TRUCK", "BUS", "BICYCLE", "PEDESTRIAN"] | list of class names for model outputs                                 |
| `model_params.point_feature_size`      | int          | `4`                                              | number of features per point in the point cloud                       |
| `model_params.max_voxel_size`          | int          | `40000`                                          | maximum number of voxels                                              |
| `model_params.point_cloud_range`       | list[double] | [-76.8, -76.8, -4.0, 76.8, 76.8, 6.0]            | detection range [min_x, min_y, min_z, max_x, max_y, max_z] [m]        |
| `model_params.voxel_size`              | list[double] | [0.32, 0.32, 10.0]                               | size of each voxel [x, y, z] [m]                                      |
| `model_params.downsample_factor`       | int          | `1`                                              | downsample factor for coordinates                                     |
| `model_params.encoder_in_feature_size` | int          | `9`                                              | number of input features to the encoder                               |
| `model_params.has_variance`            | bool         | `false`                                          | true if the model outputs pose variance as well as pose for each bbox |
| `model_params.has_twist`               | bool         | `false`                                          | true if the model outputs velocity as well as pose for each bbox      |

### Core Parameters

| Name                                             | Type         | Default Value             | Description                                                   |
| ------------------------------------------------ | ------------ | ------------------------- | ------------------------------------------------------------- |
| `encoder_onnx_path`                              | string       | `""`                      | path to VoxelFeatureEncoder ONNX file                         |
| `encoder_engine_path`                            | string       | `""`                      | path to VoxelFeatureEncoder TensorRT Engine file              |
| `head_onnx_path`                                 | string       | `""`                      | path to DetectionHead ONNX file                               |
| `head_engine_path`                               | string       | `""`                      | path to DetectionHead TensorRT Engine file                    |
| `build_only`                                     | bool         | `false`                   | shutdown the node after TensorRT engine file is built         |
| `trt_precision`                                  | string       | `fp16`                    | TensorRT inference precision: `fp32` or `fp16`                |
| `post_process_params.score_threshold`            | double       | `0.4`                     | detected objects with score less than threshold are ignored   |
| `post_process_params.yaw_norm_thresholds`        | list[double] | [0.3, 0.3, 0.3, 0.3, 0.0] | An array of distance threshold values of norm of yaw [rad].   |
| `post_process_params.iou_nms_target_class_names` | list[string] | -                         | target classes for IoU-based Non Maximum Suppression          |
| `post_process_params.iou_nms_search_distance_2d` | double       | -                         | If two objects are farther than the value, NMS isn't applied. |
| `post_process_params.iou_nms_threshold`          | double       | -                         | IoU threshold for the IoU-based Non Maximum Suppression       |
| `post_process_params.has_twist`                  | boolean      | false                     | Indicates whether the model outputs twist value.              |
| `densification_params.world_frame_id`            | string       | `map`                     | the world frame id to fuse multi-frame pointcloud             |
| `densification_params.num_past_frames`           | int          | `1`                       | the number of past frames to fuse with the current frame      |

### The `build_only` option

The `autoware_lidar_centerpoint` node has `build_only` option to build the TensorRT engine file from the ONNX file.
Although it is preferred to move all the ROS parameters in `.param.yaml` file in Autoware Universe, the `build_only` option is not moved to the `.param.yaml` file for now, because it may be used as a flag to execute the build as a pre-task. You can execute with the following command:

```bash
ros2 launch autoware_lidar_centerpoint lidar_centerpoint.launch.xml model_name:=centerpoint_tiny model_path:=/home/autoware/autoware_data/lidar_centerpoint model_param_path:=$(ros2 pkg prefix autoware_lidar_centerpoint --share)/config/centerpoint_tiny.param.yaml build_only:=true
```

## Assumptions / Known limits

- The `object.existence_probability` is stored the value of classification confidence of a DNN, not probability.

## Trained Models

You can download the onnx format of trained models by clicking on the links below.

- Centerpoint: [pts_voxel_encoder_centerpoint.onnx](https://awf.ml.dev.web.auto/perception/models/centerpoint/v2/pts_voxel_encoder_centerpoint.onnx), [pts_backbone_neck_head_centerpoint.onnx](https://awf.ml.dev.web.auto/perception/models/centerpoint/v2/pts_backbone_neck_head_centerpoint.onnx)
- Centerpoint tiny: [pts_voxel_encoder_centerpoint_tiny.onnx](https://awf.ml.dev.web.auto/perception/models/centerpoint/v2/pts_voxel_encoder_centerpoint_tiny.onnx), [pts_backbone_neck_head_centerpoint_tiny.onnx](https://awf.ml.dev.web.auto/perception/models/centerpoint/v2/pts_backbone_neck_head_centerpoint_tiny.onnx)

`Centerpoint` was trained in `nuScenes` (~28k lidar frames) [8] and TIER IV's internal database (~11k lidar frames) for 60 epochs.
`Centerpoint tiny` was trained in `Argoverse 2` (~110k lidar frames) [9] and TIER IV's internal database (~11k lidar frames) for 20 epochs.

## Training CenterPoint Model and Deploying to the Autoware

### Overview

This guide provides instructions on training a CenterPoint model using the **mmdetection3d** repository
and seamlessly deploying it within Autoware.

### Installation

#### Install prerequisites

**Step 1.** Download and install Miniconda from the [official website](https://mmpretrain.readthedocs.io/en/latest/get_started.html).

**Step 2.** Create a conda virtual environment and activate it

```bash
conda create --name train-centerpoint python=3.8 -y
conda activate train-centerpoint
```

**Step 3.** Install PyTorch

Please ensure you have PyTorch installed, and compatible with CUDA 11.6, as it is a requirement for current Autoware.

```bash
conda install pytorch==1.13.1 torchvision==0.14.1 pytorch-cuda=11.6 -c pytorch -c nvidia
```

#### Install mmdetection3d

**Step 1.** Install MMEngine, MMCV, and MMDetection using MIM

```bash
pip install -U openmim
mim install mmengine
mim install 'mmcv>=2.0.0rc4'
mim install 'mmdet>=3.0.0rc5, <3.3.0'
```

**Step 2.** Install mmdetection3d forked repository

Introduced several valuable enhancements in our fork of the mmdetection3d repository.
Notably, we've made the PointPillar z voxel feature input optional to maintain compatibility with the original paper.
In addition, we've integrated a PyTorch to ONNX converter and a T4 format reader for added functionality.

```bash
git clone https://github.com/autowarefoundation/mmdetection3d.git
cd mmdetection3d
pip install -v -e .
```

#### Use Training Repository with Docker

Alternatively, you can use Docker to run the mmdetection3d repository. We provide a Dockerfile to build a Docker image with the mmdetection3d repository and its dependencies.

Clone fork of the mmdetection3d repository

```bash
git clone https://github.com/autowarefoundation/mmdetection3d.git
```

Build the Docker image by running the following command:

```bash
cd mmdetection3d
docker build -t mmdetection3d -f docker/Dockerfile .
```

Run the Docker container:

```bash
docker run --gpus all --shm-size=8g -it -v {DATA_DIR}:/mmdetection3d/data mmdetection3d
```

### Preparing NuScenes dataset for training

**Step 1.** Download the NuScenes dataset from the [official website](https://www.nuscenes.org/download) and extract the dataset to a folder of your choice.

**Note:** The NuScenes dataset is large and requires significant disk space. Ensure you have enough storage available before proceeding.

**Step 2.** Create a symbolic link to the dataset folder

```bash
ln -s /path/to/nuscenes/dataset/ /path/to/mmdetection3d/data/nuscenes/
```

**Step 3.** Prepare the NuScenes data by running:

```bash
cd mmdetection3d
python tools/create_data.py nuscenes --root-path ./data/nuscenes --out-dir ./data/nuscenes --extra-tag nuscenes
```

### Training CenterPoint with NuScenes Dataset

#### Prepare the config file

The configuration file that illustrates how to train the CenterPoint model with the NuScenes dataset is
located at `mmdetection3d/projects/AutowareCenterPoint/configs`. This configuration file is a derived version of
[this centerpoint configuration file](https://github.com/autowarefoundation/mmdetection3d/blob/5c0613be29bd2e51771ec5e046d89ba3089887c7/configs/centerpoint/centerpoint_pillar02_second_secfpn_head-circlenms_8xb4-cyclic-20e_nus-3d.py)
from mmdetection3D.
In this custom configuration, the **use_voxel_center_z parameter** is set as **False** to deactivate the z coordinate of the voxel center,
aligning with the original paper's specifications and making the model compatible with Autoware. Additionally, the filter size is set as **[32, 32]**.

The CenterPoint model can be tailored to your specific requirements by modifying various parameters within the configuration file.
This includes adjustments related to preprocessing operations, training, testing, model architecture, dataset, optimizer, learning rate scheduler, and more.

#### Start training

```bash
python tools/train.py projects/AutowareCenterPoint/configs/centerpoint_custom.py --work-dir ./work_dirs/centerpoint_custom
```

#### Evaluation of the trained model

For evaluation purposes, we have included a sample dataset captured from the vehicle which consists of the following LiDAR sensors:
1 x Velodyne VLS128, 4 x Velodyne VLP16, and 1 x Robosense RS Bpearl. This dataset comprises 600 LiDAR frames and encompasses 5 distinct classes, 6905 cars, 3951 pedestrians,
75 cyclists, 162 buses, and 326 trucks 3D annotation. In the sample dataset, frames are annotated as 2 frames for each second. You can employ this dataset for a wide range of purposes,
including training, evaluation, and fine-tuning of models. It is organized in the T4 format.

##### Download the sample dataset

```bash
wget https://autoware-files.s3.us-west-2.amazonaws.com/dataset/lidar_detection_sample_dataset.tar.gz
#Extract the dataset to a folder of your choice
tar -xvf lidar_detection_sample_dataset.tar.gz
#Create a symbolic link to the dataset folder
ln -s /PATH/TO/DATASET/ /PATH/TO/mmdetection3d/data/tier4_dataset/
```

##### Prepare dataset and evaluate trained model

Create `.pkl` files for training, evaluation, and testing.

The dataset was formatted according to T4Dataset specifications, with 'sample_dataset' designated as one of its versions.

```bash
python tools/create_data.py T4Dataset --root-path data/sample_dataset/ --out-dir data/sample_dataset/ --extra-tag T4Dataset --version sample_dataset --annotation-hz 2
```

Run evaluation

```bash
python tools/test.py projects/AutowareCenterPoint/configs/centerpoint_custom_test.py /PATH/OF/THE/CHECKPOINT  --task lidar_det
```

Evaluation results could be relatively low because of the e to variations in sensor modalities between the sample dataset
and the training dataset. The model's training parameters are originally tailored to the NuScenes dataset, which employs a single lidar
sensor positioned atop the vehicle. In contrast, the provided sample dataset comprises concatenated point clouds positioned at
the base link location of the vehicle.

### Deploying CenterPoint model to Autoware

#### Convert CenterPoint PyTorch model to ONNX Format

The autoware_lidar_centerpoint implementation requires two ONNX models as input the voxel encoder and the backbone-neck-head of the CenterPoint model, other aspects of the network,
such as preprocessing operations, are implemented externally. Under the fork of the mmdetection3d repository,
we have included a script that converts the CenterPoint model to Autoware compatible ONNX format.
You can find it in `mmdetection3d/projects/AutowareCenterPoint` file.

```bash
python projects/AutowareCenterPoint/centerpoint_onnx_converter.py --cfg projects/AutowareCenterPoint/configs/centerpoint_custom.py --ckpt work_dirs/centerpoint_custom/YOUR_BEST_MODEL.pth --work-dir ./work_dirs/onnx_models
```

#### Create the config file for the custom model

Create a new config file named **centerpoint_custom.param.yaml** under the config file directory of the autoware_lidar_centerpoint node. Sets the parameters of the config file like
point_cloud_range, point_feature_size, voxel_size, etc. according to the training config file.

```yaml
/**:
  ros__parameters:
    class_names: ["CAR", "TRUCK", "BUS", "BICYCLE", "PEDESTRIAN"]
    point_feature_size: 4
    max_voxel_size: 40000
    point_cloud_range: [-51.2, -51.2, -3.0, 51.2, 51.2, 5.0]
    voxel_size: [0.2, 0.2, 8.0]
    downsample_factor: 1
    encoder_in_feature_size: 9
    # post-process params
    circle_nms_dist_threshold: 0.5
    iou_nms_target_class_names: ["CAR"]
    iou_nms_search_distance_2d: 10.0
    iou_nms_threshold: 0.1
    yaw_norm_thresholds: [0.3, 0.3, 0.3, 0.3, 0.0]
```

#### Launch the lidar_centerpoint node

```bash
cd /YOUR/AUTOWARE/PATH/Autoware
source install/setup.bash
ros2 launch autoware_lidar_centerpoint lidar_centerpoint.launch.xml  model_name:=centerpoint_custom  model_path:=/PATH/TO/ONNX/FILE/
```

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

## Acknowledgment: deepen.ai's 3D Annotation Tools Contribution

Special thanks to [Deepen AI](https://www.deepen.ai/) for providing their 3D Annotation tools, which have been instrumental in creating our sample dataset.

## Legal Notice

_The nuScenes dataset is released publicly for non-commercial use under the Creative
Commons Attribution-NonCommercial-ShareAlike 4.0 International Public License.
Additional Terms of Use can be found at <https://www.nuscenes.org/terms-of-use>.
To inquire about a commercial license please contact <nuscenes@motional.com>._
