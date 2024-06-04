# roi_detected_object_fusion

## Purpose

The `roi_detected_object_fusion` is a package to overwrite labels of detected objects with that of Region Of Interests (ROIs) by a 2D object detector.

## Inner-workings / Algorithms

In what follows, we describe the algorithm utilized by `roi_detected_object_fusion` (the meaning of each parameter can be found in the `Parameters` section):

1. If the `existence_probability` of a detected object is greater than the threshold, it is accepted without any further processing and published in `output`.
2. The remaining detected objects are projected onto image planes, and if the resulting ROIs overlap with the ones from the 2D detector, they are published as fused objects in `output`. The Intersection over Union (IoU) is used to determine if there are overlaps between the detections from `input` and the ROIs from `input/rois`.

The DetectedObject has three possible shape choices/implementations, where the polygon's vertices for each case are defined as follows:

- `BOUNDING_BOX`: The 8 corners of a bounding box.
- `CYLINDER`: The circle is approximated by a hexagon.
- `POLYGON`: Not implemented yet.

## Inputs / Outputs

### Input

| Name                     | Type                                                     | Description                                                |
| ------------------------ | -------------------------------------------------------- | ---------------------------------------------------------- |
| `input`                  | `autoware_perception_msgs::msg::DetectedObjects`         | input detected objects                                     |
| `input/camera_info[0-7]` | `sensor_msgs::msg::CameraInfo`                           | camera information to project 3d points onto image planes. |
| `input/rois[0-7]`        | `tier4_perception_msgs::msg::DetectedObjectsWithFeature` | ROIs from each image.                                      |
| `input/image_raw[0-7]`   | `sensor_msgs::msg::Image`                                | images for visualization.                                  |

### Output

| Name                      | Type                                             | Description                |
| ------------------------- | ------------------------------------------------ | -------------------------- |
| `output`                  | `autoware_perception_msgs::msg::DetectedObjects` | detected objects           |
| `~/debug/image_raw[0-7]`  | `sensor_msgs::msg::Image`                        | images for visualization,  |
| `~/debug/fused_objects`   | `autoware_perception_msgs::msg::DetectedObjects` | fused detected objects     |
| `~/debug/ignored_objects` | `autoware_perception_msgs::msg::DetectedObjects` | not fused detected objects |

## Parameters

### Core Parameters

| Name                                             | Type           | Description                                                                                                                |
| ------------------------------------------------ | -------------- | -------------------------------------------------------------------------------------------------------------------------- |
| `rois_number`                                    | int            | the number of input rois                                                                                                   |
| `debug_mode`                                     | bool           | If set to `true`, the node subscribes to the image topic and publishes an image with debug drawings.                       |
| `passthrough_lower_bound_probability_thresholds` | vector[double] | If the `existence_probability` of a detected object is greater than the threshold, it is published in output.              |
| `trust_distances`                                | vector[double] | If the distance of a detected object from the origin of frame_id is greater than the threshold, it is published in output. |
| `min_iou_threshold`                              | double         | If the iou between detected objects and rois is greater than `min_iou_threshold`, the objects are classified as fused.     |
| `use_roi_probability`                            | float          | If set to `true`, the algorithm uses `existence_probability` of ROIs to match with the that of detected objects.           |
| `roi_probability_threshold`                      | double         | If the `existence_probability` of ROIs is greater than the threshold, matched detected objects are published in `output`.  |
| `can_assign_matrix`                              | vector[int]    | association matrix between rois and detected_objects to check that two rois on images can be match                         |

## Assumptions / Known limits

`POLYGON`, which is a shape of a detected object, isn't supported yet.
