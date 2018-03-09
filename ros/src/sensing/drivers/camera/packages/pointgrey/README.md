# autoware_pointgrey_drivers package

This package allows to obtain an image stream from PointGrey cameras.
It was tested successfully on Grasshopper3 and LadyBug5 devices.

## Requirements
Install the SDK provided by PointGrey.

---

### grasshopper3_camera

#### How to launch
* From a sourced terminal:

`roslaunch autoware_pointgrey_drivers grasshopper3.launch`

* From Runtime manager

Sensing Tab -> Cameras -> PointGrey Grasshopper3

#### Parameters

Launch file available params.

|Parameter| Type| Description|
----------|-----|--------
|`fps`|*Integer* |Frames per second to try to acquire the image stream.|
|`mode`|*Integer*|Camera Mode, please check your camera specific valid modes. (0,1,2,...,31) |
|`format`|*String*|Pixel Format, this can be either `raw` or `rgb`. `raw` will publish the default bayer format according to your camera sensor. Both modes have 8 bit per pixel per channel.|
|`CalibrationFile`|*String*|Path to an Autoware compatible calibration file to be published in the `camera_info` topic related to this camera.|

### ladybug_camera

#### How to launch
* From a sourced terminal:

`roslaunch autoware_pointgrey_drivers ladybug.launch`

* From Runtime manager

Sensing Tab -> Cameras -> PointGrey Ladybug5

#### Parameters

Launch file available params.

|Parameter| Type| Description|
----------|-----|--------
|`SCALE`|*Float*|Floating Number between 0.1 and 1.0. Sets the downscale ratio.|
|`CalibrationFile`|*String*|Path to an Autoware compatible calibration file to be published in the `camera_info` topic related to this camera.|


### Notes

* The SDK needs to be obtained from PointGrey's website.