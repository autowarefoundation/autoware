# yabloc_pose_initializer

This package contains a node related to initial pose estimation.

- [camera_pose_initializer](#camera_pose_initializer)

This package requires the pre-trained semantic segmentation model for runtime. This model is usually downloaded by `ansible` during env preparation phase of the [installation](https://autowarefoundation.github.io/autoware-documentation/main/installation/autoware/source-installation/).
It is also possible to download it manually. Even if the model is not downloaded, initialization will still complete, but the accuracy may be compromised.

To download and extract the model manually:

```bash
$ mkdir -p ~/autoware_data/yabloc_pose_initializer/
$ wget -P ~/autoware_data/yabloc_pose_initializer/ \
       https://s3.ap-northeast-2.wasabisys.com/pinto-model-zoo/136_road-segmentation-adas-0001/resources.tar.gz
$ tar xzf ~/autoware_data/yabloc_pose_initializer/resources.tar.gz -C ~/autoware_data/yabloc_pose_initializer/
```

## Note

This package makes use of external code. The trained files are provided by apollo. The trained files are automatically downloaded during env preparation.

Original model URL

<https://github.com/openvinotoolkit/open_model_zoo/tree/master/models/intel/road-segmentation-adas-0001>

> Open Model Zoo is licensed under Apache License Version 2.0.

Converted model URL

<https://github.com/PINTO0309/PINTO_model_zoo/tree/main/136_road-segmentation-adas-0001>

> model conversion scripts are released under the MIT license

## Special thanks

- [openvinotoolkit/open_model_zoo](https://github.com/openvinotoolkit/open_model_zoo)
- [PINTO0309](https://github.com/PINTO0309)

## camera_pose_initializer

### Purpose

- This node estimates the initial position using the camera at the request of ADAPI.

#### Input

| Name                | Type                                    | Description              |
| ------------------- | --------------------------------------- | ------------------------ |
| `input/camera_info` | `sensor_msgs::msg::CameraInfo`          | undistorted camera info  |
| `input/image_raw`   | `sensor_msgs::msg::Image`               | undistorted camera image |
| `input/vector_map`  | `autoware_map_msgs::msg::LaneletMapBin` | vector map               |

#### Output

| Name                | Type                                   | Description             |
| ------------------- | -------------------------------------- | ----------------------- |
| `output/candidates` | `visualization_msgs::msg::MarkerArray` | initial pose candidates |

### Parameters

{{ json_to_markdown("localization/yabloc/yabloc_pose_initializer/schema/camera_pose_initializer.schema.json") }}

### Services

| Name               | Type                                                      | Description                     |
| ------------------ | --------------------------------------------------------- | ------------------------------- |
| `yabloc_align_srv` | `tier4_localization_msgs::srv::PoseWithCovarianceStamped` | initial pose estimation request |
