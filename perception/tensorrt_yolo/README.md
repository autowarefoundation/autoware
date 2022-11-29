# tensorrt_yolo

## Purpose

This package detects 2D bounding boxes for target objects e.g., cars, trucks, bicycles, and pedestrians on a image based on YOLO(You only look once) model.

## Inner-workings / Algorithms

### Cite

<!-- cspell:ignore Redmon, Farhadi, Bochkovskiy, Jocher, ultralytics, Roboflow, Zenodo -->

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

| Name          | Type                                               | Description                                        |
| ------------- | -------------------------------------------------- | -------------------------------------------------- |
| `out/objects` | `tier4_perception_msgs/DetectedObjectsWithFeature` | The detected objects with 2D bounding boxes        |
| `out/image`   | `sensor_msgs/Image`                                | The image with 2D bounding boxes for visualization |

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
| `gpu_id`                | int    | 0             | GPU device ID that runs the model                                  |

## Assumptions / Known limits

This package includes multiple licenses.

## Onnx model

All YOLO ONNX models are converted from the officially trained model. If you need information about training datasets and conditions, please refer to the official repositories.

All models are downloaded automatically when building. When launching the node with a model for the first time, the model is automatically converted to TensorRT, although this may take some time.

### YOLOv3

[YOLOv3](https://awf.ml.dev.web.auto/perception/models/yolov3.onnx "YOLOv3"): Converted from darknet [weight file](https://pjreddie.com/media/files/yolov3.weights "weight file") and [conf file](https://github.com/pjreddie/darknet/blob/master/cfg/yolov3.cfg "conf file").

- [This code](https://github.com/wep21/yolo_onnx_converter) is used for converting darknet weight file and conf file to onnx.

### YOLOv4

[YOLOv4](https://awf.ml.dev.web.auto/perception/models/yolov4.onnx "YOLOv4"): Converted from darknet [weight file](https://github.com/AlexeyAB/darknet/releases/download/darknet_yolo_v3_optimal/yolov4.weights "weight file") and [conf file](https://github.com/AlexeyAB/darknet/blob/master/cfg/yolov4.cfg "conf file").

[YOLOv4-tiny](https://awf.ml.dev.web.auto/perception/models/yolov4-tiny.onnx "YOLOv4-tiny"): Converted from darknet [weight file](https://github.com/AlexeyAB/darknet/releases/download/darknet_yolo_v4_pre/yolov4-tiny.weights "weight file") and [conf file](https://github.com/AlexeyAB/darknet/blob/master/cfg/yolov4-tiny.cfg "conf file").

- [This code](https://github.com/wep21/yolo_onnx_converter) is used for converting darknet weight file and conf file to onnx.

### YOLOv5

Refer to [this guide](https://github.com/ultralytics/yolov5/issues/251 "guide")

- [YOLOv5s](https://awf.ml.dev.web.auto/perception/models/yolov5s.onnx "YOLOv5s")

- [YOLOv5m](https://awf.ml.dev.web.auto/perception/models/yolov5m.onnx "YOLOv5m")

- [YOLOv5l](https://awf.ml.dev.web.auto/perception/models/yolov5l.onnx "YOLOv5l")

- [YOLOv5x](https://awf.ml.dev.web.auto/perception/models/yolov5x.onnx "YOLOv5x")

## Limitations

- If you want to run multiple instances of this node for multiple cameras using "yolo.launch.xml", first of all, create a TensorRT engine by running the "tensorrt_yolo.launch.xml" launch file separately for each GPU. Otherwise, multiple instances of the node trying to create the same TensorRT engine can cause potential problems.

## Reference repositories

- <https://github.com/pjreddie/darknet>

- <https://github.com/AlexeyAB/darknet>

- <https://github.com/ultralytics/yolov5>

- <https://github.com/wang-xinyu/tensorrtx>

- <https://github.com/NVIDIA/retinanet-examples>
