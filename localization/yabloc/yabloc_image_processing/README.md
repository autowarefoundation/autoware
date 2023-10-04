# yabloc_image_processing

This package contains some executable nodes related to image processing.

- [line_segment_detector](#line_segment_detector)
- [graph_segmentation](#graph_segmentation)
- [segment_filter](#segment_filter)
- [undistort](#undistort)
- [lanelet2_overlay](#lanelet2_overlay)
- [line_segments_overlay](#line_segments_overlay)

## line_segment_detector

### Purpose

This node extract all line segments from gray scale image.

### Inputs / Outputs

#### Input

| Name              | Type                      | Description       |
| ----------------- | ------------------------- | ----------------- |
| `input/image_raw` | `sensor_msgs::msg::Image` | undistorted image |

#### Output

| Name                              | Type                            | Description                                                                                                                          |
| --------------------------------- | ------------------------------- | ------------------------------------------------------------------------------------------------------------------------------------ |
| `output/image_with_line_segments` | `sensor_msgs::msg::Image`       | image with line segments highlighted                                                                                                 |
| `output/line_segments_cloud`      | `sensor_msgs::msg::PointCloud2` | detected line segments as point cloud. each point contains x,y,z, normal_x, normal_y, normal_z and z, and normal_z are always empty. |

## graph_segmentation

### Purpose

This node extract road surface region by [graph-based-segmentation](https://docs.opencv.org/4.5.4/dd/d19/classcv_1_1ximgproc_1_1segmentation_1_1GraphSegmentation.html).

### Inputs / Outputs

#### Input

| Name              | Type                      | Description       |
| ----------------- | ------------------------- | ----------------- |
| `input/image_raw` | `sensor_msgs::msg::Image` | undistorted image |

#### Output

| Name                     | Type                      | Description                                                |
| ------------------------ | ------------------------- | ---------------------------------------------------------- |
| `output/mask_image`      | `sensor_msgs::msg::Image` | image with masked segments determined as road surface area |
| `output/segmented_image` | `sensor_msgs::msg::Image` | segmented image for visualization                          |

### Parameters

| Name                              | Type   | Description                                                        |
| --------------------------------- | ------ | ------------------------------------------------------------------ |
| `target_height_ratio`             | double | height on the image to retrieve the candidate road surface         |
| `target_candidate_box_width`      | int    | size of the square area to search for candidate road surfaces      |
| `pickup_additional_graph_segment` | bool   | if this is true, additional regions of similar color are retrieved |
| `similarity_score_threshold`      | double | threshold for picking up additional areas                          |
| `sigma`                           | double | parameters for cv::ximgproc::segmentation                          |
| `k`                               | double | parameters for cv::ximgproc::segmentation                          |
| `min_size`                        | double | parameters for cv::ximgproc::segmentation                          |

## segment_filter

### Purpose

This is a node that integrates the results of graph_segment and lsd to extract road surface markings.

### Inputs / Outputs

#### Input

| Name                        | Type                            | Description                                                |
| --------------------------- | ------------------------------- | ---------------------------------------------------------- |
| `input/line_segments_cloud` | `sensor_msgs::msg::PointCloud2` | detected line segment                                      |
| `input/mask_image`          | `sensor_msgs::msg::Image`       | image with masked segments determined as road surface area |
| `input/camera_info`         | `sensor_msgs::msg::CameraInfo`  | undistorted camera info                                    |

#### Output

| Name                                   | Type                            | Description                                        |
| -------------------------------------- | ------------------------------- | -------------------------------------------------- |
| `output/line_segments_cloud`           | `sensor_msgs::msg::PointCloud2` | filtered line segments for visualization           |
| `output/projected_image`               | `sensor_msgs::msg::Image`       | projected filtered line segments for visualization |
| `output/projected_line_segments_cloud` | `sensor_msgs::msg::PointCloud2` | projected filtered line segments                   |

### Parameters

| Name                                   | Type   | Description                                                         |
| -------------------------------------- | ------ | ------------------------------------------------------------------- |
| `min_segment_length`                   | double | min length threshold (if it is negative, it is unlimited)           |
| `max_segment_distance`                 | double | max distance threshold (if it is negative, it is unlimited)         |
| `max_lateral_distance`                 | double | max lateral distance threshold (if it is negative, it is unlimited) |
| `publish_image_with_segment_for_debug` | bool   | toggle whether to publish the filtered line segment for debug       |
| `max_range`                            | double | range of debug projection visualization                             |
| `image_size`                           | int    | image size of debug projection visualization                        |

## undistort

### Purpose

This node performs image resizing and undistortion at the same time.

### Inputs / Outputs

#### Input

| Name                         | Type                                | Description             |
| ---------------------------- | ----------------------------------- | ----------------------- |
| `input/camera_info`          | `sensor_msgs::msg::CameraInfo`      | camera info             |
| `input/image_raw`            | `sensor_msgs::msg::Image`           | raw camera image        |
| `input/image_raw/compressed` | `sensor_msgs::msg::CompressedImage` | compressed camera image |

This node subscribes to both compressed image and raw image topics.
If raw image is subscribed to even once, compressed image will no longer be subscribed to.
This is to avoid redundant decompression within Autoware.

#### Output

| Name                 | Type                                | Description                   |
| -------------------- | ----------------------------------- | ----------------------------- |
| `output/camera_info` | `sensor_msgs::msg::CameraInfo`      | resized camera info           |
| `output/image_raw`   | `sensor_msgs::msg::CompressedImage` | undistorted and resized image |

### Parameters

| Name                | Type   | Description                                                                                    |
| ------------------- | ------ | ---------------------------------------------------------------------------------------------- |
| `use_sensor_qos`    | bool   | where to use sensor qos or not                                                                 |
| `width`             | int    | resized image width size                                                                       |
| `override_frame_id` | string | value for overriding the camera's frame_id. if blank, frame_id of static_tf is not overwritten |

#### about tf_static overriding

<details><summary>click to open</summary><div>

Some nodes requires `/tf_static` from `/base_link` to the frame_id of `/sensing/camera/traffic_light/image_raw/compressed` (e.g. `/traffic_light_left_camera/camera_optical_link`).
You can verify that the tf_static is correct with the following command.

```shell
ros2 run tf2_ros tf2_echo base_link traffic_light_left_camera/camera_optical_link
```

If the wrong `/tf_static` are broadcasted due to using a prototype vehicle, not having accurate calibration data, or some other unavoidable reason, it is useful to give the frame_id in `override_camera_frame_id`.
If you give it a non-empty string, `/image_processing/undistort_node` will rewrite the frame_id in camera_info.
For example, you can give a different tf_static as follows.

```shell
ros2 launch yabloc_launch sample_launch.xml override_camera_frame_id:=fake_camera_optical_link
ros2 run tf2_ros static_transform_publisher \
  --frame-id base_link \
  --child-frame-id fake_camera_optical_link \
  --roll -1.57 \
  --yaw -1.570
```

</div></details>

## lanelet2_overlay

### Purpose

This node overlays lanelet2 on the camera image based on the estimated self-position.

### Inputs / Outputs

#### Input

| Name                                  | Type                               | Description                                         |
| ------------------------------------- | ---------------------------------- | --------------------------------------------------- |
| `input/pose`                          | `geometry_msgs::msg::PoseStamped`  | estimated self pose                                 |
| `input/projected_line_segments_cloud` | `sensor_msgs::msg::PointCloud2`    | projected line segments including non-road markings |
| `input/camera_info`                   | `sensor_msgs::msg::CameraInfo`     | undistorted camera info                             |
| `input/image_raw`                     | `sensor_msgs::msg::Image`          | undistorted camera image                            |
| `input/ground`                        | `std_msgs::msg::Float32MultiArray` | ground tilt                                         |
| `input/ll2_road_marking`              | `sensor_msgs::msg::PointCloud2`    | lanelet2 elements regarding road surface markings   |
| `input/ll2_sign_board`                | `sensor_msgs::msg::PointCloud2`    | lanelet2 elements regarding traffic sign boards     |

#### Output

| Name                            | Type                              | Description                                            |
| ------------------------------- | --------------------------------- | ------------------------------------------------------ |
| `output/lanelet2_overlay_image` | `sensor_msgs::msg::Image`         | lanelet2 overlaid image                                |
| `output/projected_marker`       | `visualization_msgs::msg::Marker` | 3d projected line segments including non-road markings |

## line_segments_overlay

### Purpose

This node visualize classified line segments on the camera image

### Inputs / Outputs

#### Input

| Name                        | Type                            | Description              |
| --------------------------- | ------------------------------- | ------------------------ |
| `input/line_segments_cloud` | `sensor_msgs::msg::PointCloud2` | classified line segments |
| `input/image_raw`           | `sensor_msgs::msg::Image`       | undistorted camera image |

#### Output

| Name                                      | Type                      | Description                          |
| ----------------------------------------- | ------------------------- | ------------------------------------ |
| `output/image_with_colored_line_segments` | `sensor_msgs::msg::Image` | image with highlighted line segments |
