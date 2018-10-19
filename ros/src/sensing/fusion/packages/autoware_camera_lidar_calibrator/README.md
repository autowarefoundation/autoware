# Autoware Camera-LiDAR Calibration Package

## How to calibrate

Camera-LiDAR calibration is performed in two steps:
1. Obtain camera intrinsics
1. Obtain camera-LiDAR extrinsics

## Camera intrinsic calibration

The intrinsics are obtained using the `autoware_camera_calibration` script, which is a fork of the official ROS calibration tool.

### How to launch
1. In a sourced terminal:\
`rosrun autoware_camera_lidar_calibrator cameracalibrator.py --square SQUARE_SIZE --size MxN image:=/image_topic`
1. Play a rosbag or stream from a camera in the selected topic name.
1. Move the checkerboard around within the field of view of the camera until the bars turn green.
1. Press the `CALIBRATE` button.
1. The output and result of the calibration will be shown in the terminal.
1. Press the `SAVE` button.
1. A file will be saved in your home directory with the name `YYYYmmdd_HHMM_autoware_camera_calibration.yaml`.

This file will contain the intrinsic calibration to rectify the image.

### Parameters available

Flag| Parameter| Type| Description|
-----|----------|-----|--------
--square|`SQUARE_SIZE`|*double* |Defines the size of the checkerboard square in meters.|
--size|`MxN`|*string* |Defines the layout size of the checkerboard (inner size).|
image:=|`image`|*string* |Topic name of the camera image source topic in `raw` format (color or b&w).|
--min_samples|`min_samples`|*integer* |Defines the minimum number of samples required to allow calibration.|
--detection|`engine`|*string*|Chessboard detection engine, default `cv2` or `matlab` |
For extra details please visit: http://www.ros.org/wiki/camera_calibration

#### Matlab checkerboard detection engine (beta)

This node additionally supports the Matlab engine for chessboard detection, which is faster and more robust than the OpenCV implementation.

1. Go to the Matlab python setup path `/PATH/TO/MATLAB/R201XY/extern/engines/python`.
1. Run `python setup.py install` to setup Matlab bindings.

To use this engine, add `--detection matlab` to the list of arguments, i.e.\
`rosrun autoware_camera_lidar_calibrator cameracalibrator.py --detection matlab --square SQUARE_SIZE --size MxN image:=/image_topic`

![Calibration](docs/camera_calibration.jpg "Autoware camera calibration")

---

## Camera-LiDAR extrinsic calibration

Camera-LiDAR extrinsic calibration is performed by clicking on corresponding points in the image and the point cloud.

This node uses `clicked_point` and `screenpoint` from the `rviz` and `image_view2` packages respectively.

### How to launch

1. Perform the intrinsic camera calibration using camera intrinsic calibration tool described above (resulting in the file `YYYYmmdd_HHMM_autoware_camera_calibration.yaml`).
1. In a sourced terminal:\
`roslaunch autoware_camera_lidar_calibrator camera_lidar_calibration.launch intrinsics_file:=/PATH/TO/YYYYmmdd_HHMM_autoware_camera_calibration.yaml image_src:=/image`
1. An image viewer will be displayed.
1. Open Rviz and show the point cloud and the correct fixed frame.
1. Observe the image and the point cloud simultaneously.
1. Find a point within the image that you can match to a corresponding point within the point cloud.
1. Click on the pixel of the point in the image.
1. Click on the corresponding 3D point in Rviz using the *Publish Point* tool.
1. Repeat this with at least 9 different points.
1. Once finished, a file will be saved in your home directory with the name
`YYYYmmdd_HHMM_autoware_lidar_camera_calibration.yaml`.

This file can be used with Autoware's Calibration Publisher to publish and register the transformation
between the LiDAR and camera. The file contains both the intrinsic and extrinsic parameters.

### Parameters available

Parameter| Type| Description|
----------|-----|--------
|`image_src`|*string* |Topic name of the camera image source topic. Default: `/image_raw`.|
|`camera_id`|*string* |If working with more than one camera, set this to the correct camera namespace, i.e. `/camera0`.|
|`intrinsics_file`|*string* |Topic name of the camera image source topic in `raw` format (color or b&w).|
|`compressed_stream`|*bool* |If set to true, a node to convert the image from a compressed stream to an uncompressed one will be launched.|

### Camera-LiDAR calibration example

To test the calibration results, the generated yaml file can be used in the `Calibration Publisher` and then the `Points Image` in the **Sensing** tab.

![Calibration](docs/camera_lidar_calibration.jpg "Successful camera lidar calibration")

### Notes

This calibration tool assumes that the Velodyne is installed with the default order of axes for the Velodyne sensor.
* X axis points to the front
* Y axis points to the left
* Z axis points upwards
