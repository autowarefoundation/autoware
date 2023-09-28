# traffic_light_ssd_fine_detector

## Purpose

It is a package for traffic light detection using MobileNetV2 and SSDLite.

## Training Information

_NOTE_:

- Currently, Autoware supports SSD trained both [pytorch-ssd](https://github.com/qfgaohao/pytorch-ssd) and [mmdetection](https://github.com/open-mmlab/mmdetection.git).
  - Please specify either `pytorch` or `mmdetection` in `dnn_header_type`.
- Note that, tensor names of your onnx model conform the following.
  - Input tensor: `input`
  - Output box tensor: `boxes`.
  - Output score tensor: `scores`.

### Pretrained Model

The model is based on [pytorch-ssd](https://github.com/qfgaohao/pytorch-ssd) and the pretrained model could be downloaded from [here](https://drive.google.com/file/d/1puI6ltKZKJ4RoiCO-ypivzEysHaDVBsa/view).

### Training Data

The model was fine-tuned on 1750 TIER IV internal images of Japanese traffic lights.

### Trained Onnx model

- <https://drive.google.com/uc?id=1USFDPRH9JrVdGoqt27qHjRgittwc0kcO>

### Customization of CNN model

In order to train models and export onnx model, we recommend [open-mmlab/mmdetection](https://github.com/open-mmlab/mmdetection.git).
Please follow the [official document](https://mmdetection.readthedocs.io/en/latest/) to install and experiment with mmdetection. If you get into troubles, [FAQ page](https://mmdetection.readthedocs.io/en/stable/notes/faq.html) would help you.

The following steps are example of a quick-start.

#### step 0. Install [MMCV](https://github.com/open-mmlab/mmcv.git) and [MIM](https://github.com/open-mmlab/mim.git)

_NOTE_ :

- First of all, install [PyTorch](https://pytorch.org/) suitable for your CUDA version (CUDA11.6 is supported in Autoware).
- Our tested library versions are following. If the scripts as shown below would be old, please update to be suited to your version.
  - MMCV == 1.x
  - MMDetection == 2.x
  - MMDeploy == 0.x
  - MIM == 0.3.x

In order to install mmcv suitable for your CUDA version, install it specifying a url.

```shell
# Install mim
$ pip install -U openmim

# Install mmcv on a machine with CUDA11.6 and PyTorch1.13.0
$ pip install mmcv-full -f https://download.openmmlab.com/mmcv/dist/cu116/torch1.13/index.html
```

#### step 1. Install MMDetection

You can install mmdetection as a Python package or from source.

```shell
# As a Python package
$ pip install mmdet

# From source
$ git clone https://github.com/open-mmlab/mmdetection.git
$ cd mmdetection
$ pip install -v -e .
```

#### step 2. Train your model

Train model with your experiment configuration file. For the details of config file, see [here](https://mmdetection.readthedocs.io/en/latest/user_guides/config.html).

```shell
# [] is optional, you can start training from pre-trained checkpoint
$ mim train mmdet YOUR_CONFIG.py [--resume-from YOUR_CHECKPOINT.pth]
```

#### step 3. Export onnx model

In exporting onnx, use `mmdetection/tools/deployment/pytorch2onnx.py` or [open-mmlab/mmdeploy](https://github.com/open-mmlab/mmdeploy.git).
_NOTE_:

- Currently, autoware does not support TensorRT plugin for NMS defined by open-mmlab. Therefore, **please deploy onnx model excluding NMS layer**.

```shell
cd ~/mmdetection/tools/deployment
python3 pytorch2onnx.py YOUR_CONFIG.py ...
```

## Inner-workings / Algorithms

Based on the camera image and the global ROI array detected by `map_based_detection` node, a CNN-based detection method enables highly accurate traffic light detection.

## Inputs / Outputs

### Input

| Name            | Type                                               | Description                                      |
| --------------- | -------------------------------------------------- | ------------------------------------------------ |
| `~/input/image` | `sensor_msgs/Image`                                | The full size camera image                       |
| `~/input/rois`  | `tier4_perception_msgs::msg::TrafficLightRoiArray` | The array of ROIs detected by map_based_detector |

### Output

| Name                  | Type                                               | Description                  |
| --------------------- | -------------------------------------------------- | ---------------------------- |
| `~/output/rois`       | `tier4_perception_msgs::msg::TrafficLightRoiArray` | The detected accurate rois   |
| `~/debug/exe_time_ms` | `tier4_debug_msgs::msg::Float32Stamped`            | The time taken for inference |

## Parameters

### Core Parameters

| Name           | Type                | Default Value | Description                                                                     |
| -------------- | ------------------- | ------------- | ------------------------------------------------------------------------------- |
| `score_thresh` | double              | 0.7           | If the objectness score is less than this value, the object is ignored          |
| `mean`         | std::vector<double> | [0.5,0.5,0.5] | Average value of the normalized values of the image data used for training      |
| `std`          | std::vector<double> | [0.5,0.5,0.5] | Standard deviation of the normalized values of the image data used for training |

### Node Parameters

| Name               | Type   | Default Value                                                            | Description                                                          |
| ------------------ | ------ | ------------------------------------------------------------------------ | -------------------------------------------------------------------- |
| `data_path`        | string | "$(env HOME)/autoware_data"                                              | packages data and artifacts directory path                           |
| `onnx_file`        | string | "$(var data_path)/traffic_light_ssd_fine_detector/mb2-ssd-lite-tlr.onnx" | The onnx file name for yolo model                                    |
| `label_file`       | string | "$(var data_path)/traffic_light_ssd_fine_detector/voc_labels_tl.txt"     | The label file with label names for detected objects written on it   |
| `dnn_header_type`  | string | "pytorch"                                                                | Name of DNN trained toolbox: "pytorch" or "mmdetection"              |
| `mode`             | string | "FP32"                                                                   | The inference mode: "FP32", "FP16", "INT8"                           |
| `max_batch_size`   | int    | 8                                                                        | The size of the batch processed at one time by inference by TensorRT |
| `approximate_sync` | bool   | false                                                                    | Flag for whether to ues approximate sync policy                      |
| `build_only`       | bool   | false                                                                    | shutdown node after TensorRT engine file is built                    |

## Assumptions / Known limits

## Reference repositories

pytorch-ssd github repository

- <https://github.com/qfgaohao/pytorch-ssd>

MobileNetV2

- M. Sandler, A. Howard, M. Zhu, A. Zhmoginov and L. Chen, "MobileNetV2: Inverted Residuals and Linear Bottlenecks," 2018 IEEE/CVF Conference on Computer Vision and Pattern Recognition, Salt Lake City, UT, 2018, pp. 4510-4520, doi: 10.1109/CVPR.2018.00474.
