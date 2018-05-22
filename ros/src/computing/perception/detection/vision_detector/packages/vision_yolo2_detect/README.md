# How to Install Darknet for the Yolo2 Node in Autoware

1. Clone Darknet in your home directory (or modifiy the `CMakeLists.txt` file from cv_tracker):
```
% git clone https://github.com/pjreddie/darknet.git
% cd darknet
% git checkout 56d69e73aba37283ea7b9726b81afd2f79cd1134
```
This node compatibility was checked against commit `56d69e73aba37283ea7b9726b81afd2f79cd1134`.

2. Download the pre-trained models from [https://pjreddie.com/media/files/yolo.weights](https://pjreddie.com/media/files/yolo.weights), and put it in the data folder.

3. Re-compile Autoware.
```
% ./catkin_make_release
```
4. Launch the node wither using the Autoware UI or the `yolo2.launch` file.

For extra details, check the launch file.
