# How to install Darknet for Yolo2 node in Autoware 

1. Clone Darknet in your home directory ( or modifiy the CMakeLists.txt file from cv_tracker)
```
% git clone https://github.com/pjreddie/darknet.git
```
2. Download the Pretrained models from [https://pjreddie.com/media/files/yolo.weights](https://pjreddie.com/media/files/yolo.weights), and put it in the data folder.

3. Re-compile Autoware.
```
% ./catkin_make_release
```
4. Launch the node wither using the Autoware UI or the `yolo2.launch` file.

For extra details, check the launch file.
