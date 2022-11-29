# traffic_light_ssd_fine_detector

## Purpose

It is a package for traffic light detection using MobileNetV2 and SSDLite.

## Training Information

### Pretrained Model

The model is based on [pytorch-ssd](https://github.com/qfgaohao/pytorch-ssd) and the pretrained model could be downloaded from [here](https://storage.googleapis.com/models-hao/mb2-ssd-lite-mp-0_686.pth).

### Training Data

The model was fine-tuned on 1750 TierIV internal images of Japanese traffic lights.

### Trained Onnx model

- <https://drive.google.com/uc?id=1USFDPRH9JrVdGoqt27qHjRgittwc0kcO>

## Inner-workings / Algorithms

Based on the camera image and the global ROI array detected by `map_based_detection` node, a CNN-based detection method enables highly accurate traffic light detection.

## Inputs / Outputs

### Input

| Name            | Type                                                       | Description                                      |
| --------------- | ---------------------------------------------------------- | ------------------------------------------------ |
| `~/input/image` | `sensor_msgs/Image`                                        | The full size camera image                       |
| `~/input/rois`  | `autoware_auto_perception_msgs::msg::TrafficLightRoiArray` | The array of ROIs detected by map_based_detector |

### Output

| Name                  | Type                                                       | Description                  |
| --------------------- | ---------------------------------------------------------- | ---------------------------- |
| `~/output/rois`       | `autoware_auto_perception_msgs::msg::TrafficLightRoiArray` | The detected accurate rois   |
| `~/debug/exe_time_ms` | `tier4_debug_msgs::msg::Float32Stamped`                    | The time taken for inference |

## Parameters

### Core Parameters

| Name           | Type                | Default Value | Description                                                                     |
| -------------- | ------------------- | ------------- | ------------------------------------------------------------------------------- |
| `score_thresh` | double              | 0.7           | If the objectness score is less than this value, the object is ignored          |
| `mean`         | std::vector<double> | [0.5,0.5,0.5] | Average value of the normalized values of the image data used for training      |
| `std`          | std::vector<double> | [0.5,0.5,0.5] | Standard deviation of the normalized values of the image data used for training |

### Node Parameters

| Name               | Type   | Default Value                  | Description                                                          |
| ------------------ | ------ | ------------------------------ | -------------------------------------------------------------------- |
| `onnx_file`        | string | "./data/mb2-ssd-lite-tlr.onnx" | The onnx file name for yolo model                                    |
| `label_file`       | string | "./data/voc_labels_tl.txt"     | The label file with label names for detected objects written on it   |
| `mode`             | string | "FP32"                         | The inference mode: "FP32", "FP16", "INT8"                           |
| `max_batch_size`   | int    | 8                              | The size of the batch processed at one time by inference by TensorRT |
| `approximate_sync` | bool   | false                          | Flag for whether to ues approximate sync policy                      |

## Assumptions / Known limits

## Reference repositories

pytorch-ssd github repository

- <https://github.com/qfgaohao/pytorch-ssd>

MobileNetV2

- M. Sandler, A. Howard, M. Zhu, A. Zhmoginov and L. Chen, "MobileNetV2: Inverted Residuals and Linear Bottlenecks," 2018 IEEE/CVF Conference on Computer Vision and Pattern Recognition, Salt Lake City, UT, 2018, pp. 4510-4520, doi: 10.1109/CVPR.2018.00474.
