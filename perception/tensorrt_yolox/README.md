# tensorrt_yolox

## Purpose

This package detects target objects e.g., cars, trucks, bicycles, and pedestrians on a image based on [YOLOX](https://github.com/Megvii-BaseDetection/YOLOX) model.

## Inner-workings / Algorithms

### Cite

Zheng Ge, Songtao Liu, Feng Wang, Zeming Li, Jian Sun, "YOLOX: Exceeding YOLO Series in 2021", arXiv preprint arXiv:2107.08430, 2021 [[ref](https://arxiv.org/abs/2107.08430)]

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

| Name              | Type  | Default Value | Description                                                                           |
| ----------------- | ----- | ------------- | ------------------------------------------------------------------------------------- |
| `score_threshold` | float | 0.3           | If the objectness score is less than this value, the object is ignored in yolo layer. |
| `nms_threshold`   | float | 0.7           | The IoU threshold for NMS method                                                      |

**NOTE:** These two parameters are only valid for "plain" model (described later).

### Node Parameters

| Name         | Type   | Default Value | Description                                                        |
| ------------ | ------ | ------------- | ------------------------------------------------------------------ |
| `onnx_file`  | string | ""            | The onnx file name for yolo model                                  |
| `label_file` | string | ""            | The label file with label names for detected objects written on it |
| `mode`       | string | "fp32"        | The inference mode: "fp32", "fp16", "int8"                         |

## Assumptions / Known limits

The label contained in detected 2D bounding boxes (i.e., `out/objects`) will be either one of the followings:

- CAR
- PEDESTRIAN ("PERSON" will also be categorized as "PEDESTRIAN")
- BUS
- TRUCK
- BICYCLE
- MOTORCYCLE

If other labels (case insensitive) are contained in the file specified via the `label_file` parameter,
those are labeled as `UNKNOWN`, while detected rectangles are drawn in the visualization result (`out/image`).

## Onnx model

A sample model (named `yolox-tiny.onnx`) is downloaded automatically during the build process.
To accelerate Non-maximum-suppression (NMS), which is one of the common post-process after object detection inference,
`EfficientNMS_TRT` module is attached after the ordinal YOLOX (tiny) network.
The `EfficientNMS_TRT` module contains fixed values for `score_threshold` and `nms_threshold` in it,
hence these parameters are ignored when users specify ONNX models including this module.

This package accepts both `EfficientNMS_TRT` attached ONNXs and [models published from the official YOLOX repository](https://github.com/Megvii-BaseDetection/YOLOX/tree/main/demo/ONNXRuntime#download-onnx-models) (we referred to them as "plain" models).

All models are automatically converted to TensorRT format.
These converted files will be saved in the same directory as specified ONNX files
with `.engine` filename extension and reused from the next run.
The conversion process may take a while (typically a few minutes) and the inference process is blocked
until complete the conversion, so it will take some time until detection results are published on the first run.

### Package acceptable model generation

To convert users' own model that saved in PyTorch's `pth` format into ONNX,
users can exploit the converter offered by the official repository.
For the convenience, only procedures are described below.
Please refer [the official document](https://github.com/Megvii-BaseDetection/YOLOX/tree/main/demo/ONNXRuntime#convert-your-model-to-onnx) for more detail.

#### For plain models

1. Install dependency

   ```shell
   git clone git@github.com:Megvii-BaseDetection/YOLOX.git
   cd YOLOX
   python3 setup.py develop --user
   ```

2. Convert pth into ONNX

   ```shell
   python3 tools/export_onnx.py \
     --output-name YOUR_YOLOX.onnx \
     -f YOUR_YOLOX.py \
     -c YOUR_YOLOX.pth
   ```

#### For EfficientNMS_TRT embedded models

1. Install dependency

   ```shell
   git clone git@github.com:Megvii-BaseDetection/YOLOX.git
   cd YOLOX
   python3 setup.py develop --user
   pip3 install git+ssh://git@github.com/wep21/yolox_onnx_modifier.git --user
   ```

2. Convert pth into ONNX

   ```shell
   python3 tools/export_onnx.py \
     --output-name YOUR_YOLOX.onnx \
     -f YOUR_YOLOX.py \
     -c YOUR_YOLOX.pth
     --decode_in_inference
   ```

3. Embed `EfficientNMS_TRT` to the end of YOLOX

   ```shell
   yolox_onnx_modifier YOUR_YOLOX.onnx -o YOUR_YOLOX_WITH_NMS.onnx
   ```

## Label file

A sample label file (named `label.txt`)is also downloaded automatically during the build process
(**NOTE:** This file is incompatible with models that output labels for the COCO dataset (e.g., models from the official YOLOX repository)).

This file represents the correspondence between class index (integer outputted from YOLOX network) and
class label (strings making understanding easier). This package maps class IDs (incremented from 0)
with labels according to the order in this file.

## Reference repositories

- <https://github.com/Megvii-BaseDetection/YOLOX>
- <https://github.com/wep21/yolox_onnx_modifier>
