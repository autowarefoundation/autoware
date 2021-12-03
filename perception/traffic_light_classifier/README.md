# traffic_light_classifier

## Purpose

traffic_light_classifier is a package for classifying traffic light labels using cropped image around a traffic light. This package has two classifier models: `cnn_classifier` and `hsv_classifier`.

## Inner-workings / Algorithms

### cnn_classifier

Traffic light labels are classified by MobileNetV2.

### hsv_classifier

Traffic light colors (green, yellow and red) are classified in HSV model.

### About Label

The message type is designed to comply with the unified road signs proposed at the [Vienna Convention](https://en.wikipedia.org/wiki/Vienna_Convention_on_Road_Signs_and_Signals#Traffic_lights). This idea has been also proposed in [Autoware.Auto](https://gitlab.com/autowarefoundation/autoware.auto/autoware_auto_msgs/-/merge_requests/16).

There are rules for naming labels that nodes receive. One traffic light is represented by the following character string separated by commas. `color1-shape1, color2-shape2` .

For example, the simple red and red cross traffic light label must be expressed as "red-circle, red-cross".

These colors and shapes are assigned to the message as follows:
![TrafficLightDataStructure.jpg](./image/TrafficLightDataStructure.jpg)

## Inputs / Outputs

### Input

| Name            | Type                                                       | Description            |
| --------------- | ---------------------------------------------------------- | ---------------------- |
| `~/input/image` | `sensor_msgs::msg::Image`                                  | input image            |
| `~/input/rois`  | `autoware_auto_perception_msgs::msg::TrafficLightRoiArray` | rois of traffic lights |

### Output

| Name                       | Type                                                     | Description         |
| -------------------------- | -------------------------------------------------------- | ------------------- |
| `~/output/traffic_signals` | `autoware_auto_perception_msgs::msg::TrafficSignalArray` | classified signals  |
| `~/output/debug/image`     | `sensor_msgs::msg::Image`                                | image for debugging |

## Parameters

### Node Parameters

| Name              | Type | Description                                 |
| ----------------- | ---- | ------------------------------------------- |
| `classifier_type` | int  | if the value is `1`, cnn_classifier is used |

### Core Parameters

#### cnn_classifier

| Name              | Type | Description                          |
| ----------------- | ---- | ------------------------------------ |
| `model_file_path` | str  | path to the model file               |
| `label_file_path` | str  | path to the label file               |
| `precision`       | str  | TensorRT precision, `fp16` or `int8` |
| `input_c`         | str  | the channel size of an input image   |
| `input_h`         | str  | the height of an input image         |
| `input_w`         | str  | the width of an input image          |

#### hsv_classifier

| Name           | Type | Description                                    |
| -------------- | ---- | ---------------------------------------------- |
| `green_min_h`  | int  | the minimum hue of green color                 |
| `green_min_s`  | int  | the minimum saturation of green color          |
| `green_min_v`  | int  | the minimum value (brightness) of green color  |
| `green_max_h`  | int  | the maximum hue of green color                 |
| `green_max_s`  | int  | the maximum saturation of green color          |
| `green_max_v`  | int  | the maximum value (brightness) of green color  |
| `yellow_min_h` | int  | the minimum hue of yellow color                |
| `yellow_min_s` | int  | the minimum saturation of yellow color         |
| `yellow_min_v` | int  | the minimum value (brightness) of yellow color |
| `yellow_max_h` | int  | the maximum hue of yellow color                |
| `yellow_max_s` | int  | the maximum saturation of yellow color         |
| `yellow_max_v` | int  | the maximum value (brightness) of yellow color |
| `red_min_h`    | int  | the minimum hue of red color                   |
| `red_min_s`    | int  | the minimum saturation of red color            |
| `red_min_v`    | int  | the minimum value (brightness) of red color    |
| `red_max_h`    | int  | the maximum hue of red color                   |
| `red_max_s`    | int  | the maximum saturation of red color            |
| `red_max_v`    | int  | the maximum value (brightness) of red color    |

## Assumptions / Known limits

<!-- Write assumptions and limitations of your implementation.

Example:
  This algorithm assumes obstacles are not moving, so if they rapidly move after the vehicle started to avoid them, it might collide with them.
  Also, this algorithm doesn't care about blind spots. In general, since too close obstacles aren't visible due to the sensing performance limit, please take enough margin to obstacles.
-->

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

[1] M. Sandler, A. Howard, M. Zhu, A. Zhmoginov and L. Chen, "MobileNetV2: Inverted Residuals and Linear Bottlenecks," 2018 IEEE/CVF Conference on Computer Vision and Pattern Recognition, Salt Lake City, UT, 2018, pp. 4510-4520, doi: 10.1109/CVPR.2018.00474.

## (Optional) Future extensions / Unimplemented parts

<!-- Write future extensions of this package.

Example:
  Currently, this package can't handle the chattering obstacles well. We plan to add some probabilistic filters in the perception layer to improve it.
  Also, there are some parameters that should be global(e.g. vehicle size, max steering, etc.). These will be refactored and defined as global parameters so that we can share the same parameters between different nodes.
-->
