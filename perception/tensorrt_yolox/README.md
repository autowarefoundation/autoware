# tensorrt_yolox

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

### Core Parameters

| Name              | Type  | Default Value | Description                                                                            |
| ----------------- | ----- | ------------- | -------------------------------------------------------------------------------------- |
| `score_threshold` | float | 0.3           | If the objectness score is less than this value, the object is ignored in yolox layer. |
| `nms_threshold`   | float | 0.7           | The IoU threshold for NMS method                                                       |

**NOTE:** These two parameters are only valid for "plain" model (described later).

### Node Parameters

| Name                                   | Type   | Default Value | Description                                                                                                                                                                                                                                 |
| -------------------------------------- | ------ | ------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `model_path`                           | string | ""            | The onnx file name for yolox model                                                                                                                                                                                                          |
| `model_name`                           | string | ""            | The yolox model name: <br /> "yolox-sPlus-T4-960x960-pseudo-finetune" for detection only, could reduce resource and processing_time <br /> "yolox-sPlus-opt-pseudoV2-T4-960x960-T4-seg16cls" for multi-task including semantic segmentation |
| `label_path`                           | string | ""            | The label file with label names for detected objects written on it                                                                                                                                                                          |
| `precision`                            | string | "fp16"        | The inference mode: "fp32", "fp16", "int8"                                                                                                                                                                                                  |
| `build_only`                           | bool   | false         | shutdown node after TensorRT engine file is built                                                                                                                                                                                           |
| `calibration_algorithm`                | string | "MinMax"      | Calibration algorithm to be used for quantization when precision==int8. Valid value is one of: Entropy",("Legacy" \| "Percentile"), "MinMax"]                                                                                               |
| `dla_core_id`                          | int    | -1            | If positive ID value is specified, the node assign inference task to the DLA core                                                                                                                                                           |
| `quantize_first_layer`                 | bool   | false         | If true, set the operating precision for the first (input) layer to be fp16. This option is valid only when precision==int8                                                                                                                 |
| `quantize_last_layer`                  | bool   | false         | If true, set the operating precision for the last (output) layer to be fp16. This option is valid only when precision==int8                                                                                                                 |
| `profile_per_layer`                    | bool   | false         | If true, profiler function will be enabled. Since the profile function may affect execution speed, it is recommended to set this flag true only for development purpose.                                                                    |
| `clip_value`                           | double | 0.0           | If positive value is specified, the value of each layer output will be clipped between [0.0, clip_value]. This option is valid only when precision==int8 and used to manually specify the dynamic range instead of using any calibration    |
| `preprocess_on_gpu`                    | bool   | true          | If true, pre-processing is performed on GPU                                                                                                                                                                                                 |
| `calibration_image_list_path`          | string | ""            | Path to a file which contains path to images. Those images will be used for int8 quantization.                                                                                                                                              |
| `yolox_s_plus_opt_param_path`          | string | ""            | Path to parameter file                                                                                                                                                                                                                      |
| `is_publish_color_mask`                | bool   | false         | If true, publish color mask for result visualization                                                                                                                                                                                        |
| `is_roi_overlap_segment`               | bool   | true          | If true, overlay detected object roi onto semantic segmentation to avoid over-filtering pointcloud especially small size objects                                                                                                            |
| `overlap_roi_score_threshold`          | float  | 0.3           | minimum existence_probability of detected roi considered to replace segmentation                                                                                                                                                            |
| `roi_overlay_segment_label.UNKNOWN`    | bool   | true          | If true, unknown objects roi will be overlaid onto sematic segmentation mask.                                                                                                                                                               |
| `roi_overlay_segment_label.CAR`        | bool   | false         | If true, car objects roi will be overlaid onto sematic segmentation mask.                                                                                                                                                                   |
| `roi_overlay_segment_label.TRUCK`      | bool   | false         | If true, truck objects roi will be overlaid onto sematic segmentation mask.                                                                                                                                                                 |
| `roi_overlay_segment_label.BUS`        | bool   | false         | If true, bus objects roi will be overlaid onto sematic segmentation mask.                                                                                                                                                                   |
| `roi_overlay_segment_label.MOTORCYCLE` | bool   | true          | If true, motorcycle objects roi will be overlaid onto sematic segmentation mask.                                                                                                                                                            |
| `roi_overlay_segment_label.BICYCLE`    | bool   | true          | If true, bicycle objects roi will be overlaid onto sematic segmentation mask.                                                                                                                                                               |
| `roi_overlay_segment_label.PEDESTRIAN` | bool   | true          | If true, pedestrian objects roi will be overlaid onto sematic segmentation mask.                                                                                                                                                            |
| `roi_overlay_segment_label.ANIMAL`     | bool   | true          | If true, animal objects roi will be overlaid onto sematic segmentation mask.                                                                                                                                                                |

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
