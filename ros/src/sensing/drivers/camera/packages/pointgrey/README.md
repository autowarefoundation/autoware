# Autoware Point Grey Camera Drivers Package

This package allows the capture of an image stream from Point Grey cameras.
It has been tested successfully with Grasshopper3 and LadyBug5 devices on both Ubuntu 14.04 and 16.04.

## Requirements
* FlyCapture SDK or Spinnaker SDK provided by Point Grey.

---

## Grasshopper3

### How to launch
* From a sourced terminal:\
`roslaunch autoware_pointgrey_drivers grasshopper3.launch`
* From Runtime manager:\
Sensing Tab -> Cameras -> PointGrey Grasshopper3

### Parameters available

|Parameter| Type| Description|
----------|-----|--------
|`fps`|*integer* |Defines the frames per second at which to attempt image stream acquisition.|
|`mode`|*integer*|Camera Mode - please check your camera for valid modes (0,1,2,...,31). |
|`format`|*string*|Pixel Format, which can be either `raw` or `rgb`. `raw` will publish the default bayer format according to your camera sensor. Both modes have 8 bits per pixel per channel.|
|`timeout`|*integer*|Timeout in miliseconds. Default 1000 ms.|
|`CalibrationFile`|*string*|Path to an Autoware-compatible calibration file to be published in the `camera_info` topic related to this camera.|

## Ladybug

### How to launch
* From a sourced terminal:\
`roslaunch autoware_pointgrey_drivers ladybug.launch`

* From Runtime manager:\
Sensing Tab -> Cameras -> PointGrey Ladybug5

### Parameters available

|Parameter| Type| Description|
----------|-----|--------
|`SCALE`|*float*|Defines the downscale ratio (between 0.1 and 1.0).|
|`CalibrationFile`|*string*|Path to an Autoware-compatible calibration file to be published in the `camera_info` topic related to this camera.|

### Notes

* The FlyCapture SDK must be obtained from Point Grey's website.\
<https://www.ptgrey.com/flycapture-sdk>


## FLIR ADK

Execute from `Autoware/ros` base path.

1. `rosdep update`
1. `rosdep install -y --from-paths src --ignore-src --rosdistro $ROS_DISTRO` 
1. `./catkin_make_release`
1. `./run`
1. Connect your camera
1. Confirm the camera has been detected using `v4l2-ctl --list-devices`
1. Select from *Sensing* tab / *Cameras* -> FLIR ADK
1. Press the `[config]` button and write the device name reported by `v4l2-ctl --list-devices`.
1. Click the *FLIR ADK* checkbox
1. Open Rviz and add image topic `/flir_adk/camera/image_raw`

### Parameters available

|Parameter| Type| Description|
----------|-----|--------
|`DEVICE`|*string* |Name of the system device in the form `/dev/videoX`. Obtain the correct name using `v4l2-ctl --list-devices`.|
|`FPS`|*integer*|Frame per second. Default 30. |
|`WIDTH`|*integer*|Image width of the stream (Default 640).|
|`HEIGHT`|*integer*|Image height of the camera (Default 512).|
|`NS`|*string*|Namespace to add as prefix. Default `flir_adk`.|

## Spinnaker

### How to launch
* From a sourced terminal:\
`roslaunch autoware_pointgrey_drivers spinnaker.launch`
* From Runtime manager:\
Sensing Tab -> Cameras -> PointGrey Spinnaker

### Parameters available

|Parameter| Type| Description|
----------|-----|--------
|`fps`|*integer* |Frame per second (Default 60).|
|`width`|*integer*|Image width of the stream (Default 1440).|
|`height`|*integer*|Image height of the camera (Default 1080).|
|`dltl`|*integer*|DeviceLinkThroughputLimit (Default 100000000).|


