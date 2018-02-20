## Baumer Package

This package allows to obtain an image stream from Baumer cameras.
It was testes successfully on a VLG22C.

### Requirements

Baumer SDK installed in HOME (Default path)

### How to launch

* From a sourced terminal:

`roslaunch vlg22c_cam baumer.launch`

* From Runtime manager

Sensing Tab -> Cameras -> VLG-22

### Parameters
Launch file available params.

|Parameter| Type| Description|
----------|-----|--------
|`fps`|*Integer* |Frames per second to try to acquire the image stream.|
|`scale`|*Double*|Number between 0.1 and 1.0, it will scale down the image according to this ratio. |
|`brightness`|*Double*|Number between 0.1 and 1.0, Brightness Threshold to trigger exposure compensation. The camera does not have automatic exposure parametrization, therefore a naive Histogram compensation algorithm was added. |
|`CalibrationFile`|*String*|Path to an Autoware compatible calibration file to be published in the `camera_info` topic related to this camera.|


### Notes

* The SDK needs to be obtained from Baumer's website.
* The node will only be compiled if the SDK is installed in the default directory inside ${HOME}