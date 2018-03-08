## Baumer Package

This package allows the capture of an image stream from Baumer cameras.
It was tested successfully using a VLG22C.

### Requirements

Baumer SDK installed in HOME (Default path)

### How to launch

* From a sourced terminal:

`roslaunch vlg22c_cam baumer.launch`

* From Runtime Manager:

Sensing Tab -> Cameras -> VLG-22

### Parameters

Launch file available parameters:

|Parameter| Type| Description|
----------|-----|--------
|`fps`|*Integer* |Frames per second to try to acquire the image stream.|
|`scale`|*Double*|Number between 0.1 and 1.0, the scaling factor for the output image.|
|`brightness`|*Double*|Number between 0.1 and 1.0, the brightness threshold to trigger exposure compensation. The camera does not have automatic exposure parametrization, therefore a naive Histogram compensation algorithm was added.|
|`CalibrationFile`|*String*|Path to an Autoware compatible calibration file to be published in the `camera_info` topic related to this camera.|

### Notes

* The SDK needs to be obtained from Baumer's website.
* The node will only be compiled if the SDK is installed in the default directory inside ${HOME}
