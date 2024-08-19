# autoware_tensorrt_yolox

## Purpose

This package detects target objects e.g., cars, trucks, bicycles, and pedestrians and segment target objects such as cars, trucks, buses and pedestrian, building, vegetation, road, sidewalk on a image based on [YOLOX](https://github.com/Megvii-BaseDetection/YOLOX) model with multi-header structure.

## Inner-workings / Algorithms

### Cite

<!-- cspell: ignore Zheng, Songtao, Feng, Zeming, Jian, semseg -->

Zheng Ge, Songtao Liu, Feng Wang, Zeming Li, Jian Sun, "YOLOX: Exceeding YOLO Series in 2021", arXiv preprint arXiv:2107.08430, 2021 [[ref](https://arxiv.org/abs/2107.08430)]

## Inputs / Outputs

### Input

| Name       | Type                | Description     |
| ---------- | ------------------- | --------------- |
| `in/image` | `sensor_msgs/Image` | The input image |

### Output

| Name             | Type                                               | Description                                                         |
| ---------------- | -------------------------------------------------- | ------------------------------------------------------------------- |
| `out/objects`    | `tier4_perception_msgs/DetectedObjectsWithFeature` | The detected objects with 2D bounding boxes                         |
| `out/image`      | `sensor_msgs/Image`                                | The image with 2D bounding boxes for visualization                  |
| `out/mask`       | `sensor_msgs/Image`                                | The semantic segmentation mask                                      |
| `out/color_mask` | `sensor_msgs/Image`                                | The colorized image of semantic segmentation mask for visualization |

## Parameters

{{ json_to_markdown("perception/autoware_tensorrt_yolox/schema/yolox_s_plus_opt.schema.json") }}
{{ json_to_markdown("perception/autoware_tensorrt_yolox/schema/yolox_tiny.schema.json") }}

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

The semantic segmentation mask is a gray image whose each pixel is index of one following class:

| index | semantic name    |
| ----- | ---------------- |
| 0     | road             |
| 1     | building         |
| 2     | wall             |
| 3     | obstacle         |
| 4     | traffic_light    |
| 5     | traffic_sign     |
| 6     | person           |
| 7     | vehicle          |
| 8     | bike             |
| 9     | road             |
| 10    | sidewalk         |
| 11    | roadPaint        |
| 12    | curbstone        |
| 13    | crosswalk_others |
| 14    | vegetation       |
| 15    | sky              |

## Onnx model

A sample model (named `yolox-tiny.onnx`) is downloaded by ansible script on env preparation stage, if not, please, follow [Manual downloading of artifacts](https://github.com/autowarefoundation/autoware/tree/main/ansible/roles/artifacts).
To accelerate Non-maximum-suppression (NMS), which is one of the common post-process after object detection inference,
`EfficientNMS_TRT` module is attached after the ordinal YOLOX (tiny) network.
The `EfficientNMS_TRT` module contains fixed values for `score_threshold` and `nms_threshold` in it,
hence these parameters are ignored when users specify ONNX models including this module.

This package accepts both `EfficientNMS_TRT` attached ONNXs and [models published from the official YOLOX repository](https://github.com/Megvii-BaseDetection/YOLOX/tree/main/demo/ONNXRuntime#download-onnx-models) (we referred to them as "plain" models).

In addition to `yolox-tiny.onnx`, a custom model named `yolox-sPlus-opt-pseudoV2-T4-960x960-T4-seg16cls` is either available.
This model is multi-header structure model which is based on YOLOX-s and tuned to perform more accurate detection with almost comparable execution speed with `yolox-tiny`.
To get better results with this model, users are recommended to use some specific running arguments
such as `precision:=int8`, `calibration_algorithm:=Entropy`, `clip_value:=6.0`.
Users can refer `launch/yolox_sPlus_opt.launch.xml` to see how this model can be used.
Beside detection result, this model also output image semantic segmentation result for pointcloud filtering purpose.

All models are automatically converted to TensorRT format.
These converted files will be saved in the same directory as specified ONNX files
with `.engine` filename extension and reused from the next run.
The conversion process may take a while (**typically 10 to 20 minutes**) and the inference process is blocked
until complete the conversion, so it will take some time until detection results are published (**even until appearing in the topic list**) on the first run

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

A sample label file (named `label.txt`) and semantic segmentation color map file (name `semseg_color_map.csv`) are also downloaded automatically during env preparation process
(**NOTE:** This file is incompatible with models that output labels for the COCO dataset (e.g., models from the official YOLOX repository)).

This file represents the correspondence between class index (integer outputted from YOLOX network) and
class label (strings making understanding easier). This package maps class IDs (incremented from 0)
with labels according to the order in this file.

## Reference repositories

- <https://github.com/Megvii-BaseDetection/YOLOX>
- <https://github.com/wep21/yolox_onnx_modifier>
- <https://github.com/tier4/trt-yoloXP>
