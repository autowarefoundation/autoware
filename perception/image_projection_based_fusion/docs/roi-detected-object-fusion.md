# roi_detected_object_fusion

## Purpose

The `roi_detected_object_fusion` is a package to overwrite labels of detected objects with that of Region Of Interests (ROIs) by a 2D object detector.

## Inner-workings / Algorithms

The detected objects are projected onto image planes, and then if the ROIs of detected objects (3D ROIs) and from a 2D detector (2D ROIs) are overlapped, the labels of detected objects are overwritten with that of 2D ROIs. Intersection over Union (IoU) is used to determine if there are overlaps between them.

The `DetectedObject` has three shape and the polygon vertices of a object are as below:

- `BOUNDING_BOX`: The 8 corners of a bounding box.
- `CYLINDER`: The circle is approximated by a hexagon.
- `POLYGON`: Not implemented yet.

## Inputs / Outputs

### Input

| Name                  | Type                                                     | Description                                                                        |
| --------------------- | -------------------------------------------------------- | ---------------------------------------------------------------------------------- |
| `input`               | `autoware_auto_perception_msgs::msg::DetectedObjects`    | detected objects                                                                   |
| `input/camera_infoID` | `sensor_msgs::msg::CameraInfo`                           | camera information to project 3d points onto image planes, `ID` is between 0 and 7 |
| `input/roisID`        | `tier4_perception_msgs::msg::DetectedObjectsWithFeature` | ROIs from each image, `ID` is between 0 and 7                                      |
| `input/image_rawID`   | `sensor_msgs::msg::Image`                                | images for visualization, `ID` is between 0 and 7                                  |

### Output

| Name                 | Type                                                  | Description                                       |
| -------------------- | ----------------------------------------------------- | ------------------------------------------------- |
| `output`             | `autoware_auto_perception_msgs::msg::DetectedObjects` | detected objects                                  |
| `output/image_rawID` | `sensor_msgs::msg::Image`                             | images for visualization, `ID` is between 0 and 7 |

## Parameters

### Core Parameters

| Name            | Type  | Description                                                                    |
| --------------- | ----- | ------------------------------------------------------------------------------ |
| `use_iou_x`     | bool  | calculate IoU only along x-axis                                                |
| `use_iou_y`     | bool  | calculate IoU only along y-axis                                                |
| `use_iou`       | bool  | calculate IoU both along x-axis and y-axis                                     |
| `iou_threshold` | float | the IoU threshold to overwrite a label of a detected object with that of a roi |
| `rois_number`   | int   | the number of input rois                                                       |
| `debug_mode`    | bool  | If `true`, subscribe and publish images for visualization.                     |

## Assumptions / Known limits

`POLYGON`, which is a shape of a detected object, isn't supported yet.
