# tensorrt_yolo

## Purpose

This package detects 2D bounding boxes for target objects e.g., cars, trucks, bicycles, and pedestrians on a image based on YOLO(You only look once) model.

## Inner-workings / Algorithms

### Cite

yolov3

Redmon, J., & Farhadi, A. (2018). Yolov3: An incremental improvement. arXiv preprint arXiv:1804.02767.

yolov4

Bochkovskiy, A., Wang, C. Y., & Liao, H. Y. M. (2020). Yolov4: Optimal speed and accuracy of object detection. arXiv preprint arXiv:2004.10934.

yolov5

Jocher, G., et al. (2021). ultralytics/yolov5: v6.0 - YOLOv5n 'Nano' models, Roboflow integration, TensorFlow export, OpenCV DNN support (v6.0). Zenodo. <https://doi.org/10.5281/zenodo.5563715>

## Inputs / Outputs

### Input

| Name       | Type                | Description     |
| ---------- | ------------------- | --------------- |
| `in/image` | `sensor_msgs/Image` | The input image |

### Output

| Name          | Type                                                  | Description                                        |
| ------------- | ----------------------------------------------------- | -------------------------------------------------- |
| `out/objects` | `autoware_perception_msgs/DetectedObjectsWithFeature` | The detected objects with 2D bounding boxes        |
| `out/image`   | `sensor_msgs/Image`                                   | The image with 2D bounding boxes for visualization |

## Parameters

### Core Parameters

| Name                | Type         | Default Value                                                                                                      | Description                                                                           |
| ------------------- | ------------ | ------------------------------------------------------------------------------------------------------------------ | ------------------------------------------------------------------------------------- |
| `anchors`           | double array | [10.0, 13.0, 16.0, 30.0, 33.0, 23.0, 30.0, 61.0, 62.0, 45.0, 59.0, 119.0, 116.0, 90.0, 156.0, 198.0, 373.0, 326.0] | The anchors to create bounding box candidates                                         |
| `scale_x_y`         | double array | [1.0, 1.0, 1.0]                                                                                                    | The scale parameter to eliminate grid sensitivity                                     |
| `score_thresh`      | double       | 0.1                                                                                                                | If the objectness score is less than this value, the object is ignored in yolo layer. |
| `iou_thresh`        | double       | 0.45                                                                                                               | The iou threshold for NMS method                                                      |
| `detections_per_im` | int          | 100                                                                                                                | The maximum detection number for one frame                                            |
| `use_darknet_layer` | bool         | true                                                                                                               | The flag to use yolo layer in darknet                                                 |
| `ignore_thresh`     | double       | 0.5                                                                                                                | If the output score is less than this value, ths object is ignored.                   |

### Node Parameters

| Name                    | Type   | Default Value | Description                                                        |
| ----------------------- | ------ | ------------- | ------------------------------------------------------------------ |
| `onnx_file`             | string | ""            | The onnx file name for yolo model                                  |
| `engine_file`           | string | ""            | The tensorrt engine file name for yolo model                       |
| `label_file`            | string | ""            | The label file with label names for detected objects written on it |
| `calib_image_directory` | string | ""            | The directory name including calibration images for int8 inference |
| `calib_cache_file`      | string | ""            | The calibration cache file for int8 inference                      |
| `mode`                  | string | "FP32"        | The inference mode: "FP32", "FP16", "INT8"                         |

## Assumptions / Known limits

This package includes multiple licenses.

## Onnx model

### YOLOv3

[YOLOv3](https://drive.google.com/uc?id=1ZYoBqVynmO5kKntyN56GEbELRpuXG8Ym "YOLOv3"): Converted from darknet [weight file](https://pjreddie.com/media/files/yolov3.weights "weight file") and [conf file](https://github.com/pjreddie/darknet/blob/master/cfg/yolov3.cfg "conf file").

### YOLOv4

[YOLOv4](https://drive.google.com//uc?id=1vkNmSwcIpTkJ_-BrKhxtit0PBJeJYTVX "YOLOv4"): Converted from darknet [weight file](https://github.com/AlexeyAB/darknet/releases/download/darknet_yolo_v3_optimal/yolov4.weights "weight file") and [conf file](https://github.com/AlexeyAB/darknet/blob/master/cfg/yolov4.cfg "conf file").

### YOLOv5

Refer to [this guide](https://github.com/ultralytics/yolov5/issues/251 "guide")

- [YOLOv5s](https://drive.google.com//uc?id=1CF21nQWigwCPTr5psQZXg6cBQIOYKbad "YOLOv5s")

- [YOLOv5m](https://drive.google.com//uc?id=1a1h50KJH6slwmjKZpPlS-errukF-BrgG "YOLOv5m")

- [YOLOv5l](https://drive.google.com/uc?id=1xO8S92Cq7qrmx93UHHyA7Cd7-dJsBDP8 "YOLOv5l")

- [YOLOv5x](https://drive.google.com/uc?id=1kAHuNJUCxpD-yWrS6t95H3zbAPfClLxI "YOLOv5x")

## Reference repositories

- <https://github.com/pjreddie/darknet>

- <https://github.com/AlexeyAB/darknet>

- <https://github.com/ultralytics/yolov5>

- <https://github.com/wang-xinyu/tensorrtx>

- <https://github.com/NVIDIA/retinanet-examples>
