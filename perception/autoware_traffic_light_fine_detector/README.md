# traffic_light_fine_detector

## Purpose

It is a package for traffic light detection using YoloX-s.

## Training Information

### Pretrained Model

The model is based on [YOLOX](https://github.com/Megvii-BaseDetection/YOLOX) and the pretrained model could be downloaded from [here](https://github.com/Megvii-BaseDetection/YOLOX/releases/download/0.1.1rc0/yolox_s.pth).

### Training Data

The model was fine-tuned on around 17,000 TIER IV internal images of Japanese traffic lights.

### Trained Onnx model

You can download the ONNX file using these instructions.  
Please visit [autoware-documentation](https://github.com/autowarefoundation/autoware-documentation/blob/main/docs/models/index.md) for more information.

## Inner-workings / Algorithms

Based on the camera image and the global ROI array detected by `map_based_detection` node, a CNN-based detection method enables highly accurate traffic light detection.

## Inputs / Outputs

### Input

| Name            | Type                                               | Description                                                         |
| --------------- | -------------------------------------------------- | ------------------------------------------------------------------- |
| `~/input/image` | `sensor_msgs/Image`                                | The full size camera image                                          |
| `~/input/rois`  | `tier4_perception_msgs::msg::TrafficLightRoiArray` | The array of ROIs detected by map_based_detector                    |
| `~/expect/rois` | `tier4_perception_msgs::msg::TrafficLightRoiArray` | The array of ROIs detected by map_based_detector without any offset |

### Output

| Name                  | Type                                               | Description                  |
| --------------------- | -------------------------------------------------- | ---------------------------- |
| `~/output/rois`       | `tier4_perception_msgs::msg::TrafficLightRoiArray` | The detected accurate rois   |
| `~/debug/exe_time_ms` | `tier4_debug_msgs::msg::Float32Stamped`            | The time taken for inference |

## Parameters

### Core Parameters

| Name                         | Type   | Default Value | Description                                                            |
| ---------------------------- | ------ | ------------- | ---------------------------------------------------------------------- |
| `fine_detector_score_thresh` | double | 0.3           | If the objectness score is less than this value, the object is ignored |
| `fine_detector_nms_thresh`   | double | 0.65          | IoU threshold to perform Non-Maximum Suppression                       |

### Node Parameters

| Name                       | Type    | Default Value               | Description                                                        |
| -------------------------- | ------- | --------------------------- | ------------------------------------------------------------------ |
| `data_path`                | string  | "$(env HOME)/autoware_data" | packages data and artifacts directory path                         |
| `fine_detector_model_path` | string  | ""                          | The onnx file name for yolo model                                  |
| `fine_detector_label_path` | string  | ""                          | The label file with label names for detected objects written on it |
| `fine_detector_precision`  | string  | "fp32"                      | The inference mode: "fp32", "fp16"                                 |
| `approximate_sync`         | bool    | false                       | Flag for whether to ues approximate sync policy                    |
| `gpu_id`                   | integer | 0                           | ID for the selecting CUDA GPU device                               |

## Assumptions / Known limits

## Reference repositories

YOLOX github repository

- <https://github.com/Megvii-BaseDetection/YOLOX>
