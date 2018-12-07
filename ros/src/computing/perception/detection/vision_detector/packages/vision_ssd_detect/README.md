# How to Install Caffe for SSD vision detector
**Updated as of 2018-07-04**

1. Complete the [pre-requisites](http://caffe.berkeleyvision.org/install_apt.html).

2. Clone the SSD Caffe fork in your home directory (CMake files will be looking for it there).
```
% git clone -b ssd https://github.com/weiliu89/caffe.git ssdcaffe
% cd ssdcaffe
% git checkout 4817bf8b4200b35ada8ed0dc378dceaf38c539e4
```

3. Follow the authors' instructions to complete the pre-requisites for compilation.

4. Compile Caffe:
```
make && make distribute
```

6. Download pre-trained models as provided at https://github.com/weiliu89/caffe/tree/ssd, or use your own.

Once compiled, run from the terminal, or launch from RunTimeManager:

```
% roslaunch vision_ssd_detect vision_ssd_detect network_definition_file:=/PATH/TO/deploy.prototxt pretrained_model_file:=/PATH/TO/model.caffemodel
```
Remember to modify the launch file located inside 
`AUTOWARE_BASEDIR/ros/src/computing/perception/detection/vision_detector/packages/vision_ssd_detect/launch/vision_ssd_detect.launch` 
and point the network and pre-trained models to your paths.


## Launch file params

|Parameter| Type| Description|Default|
----------|-----|--------|---|
|`use_gpu`|*Bool* |Whether to use or not GPU acceleration.|`true`|
|`gpu_device_id`|*Integer*|ID of the GPU to be used.|`0`|
|`score_threshold`|*Double*|Value between 0 and 1. Defines the minimum score value to filter detections.|`0.5`|
|`network_definition_file`|*String*|Path to the prototxt file .|`$SSDCAFFEPATH/models/VGGNet/VOC0712/SSD_512x512/deploy.prototxt`|
|`pretrained_model_file`|*String*|Path to the prototxt file .|`$SSDCAFFEPATH/models/VGGNet/VOC0712/SSD_512x512/VGG_VOC0712_SSD_512x512_iter_120000.caffemodel`|
|`camera_id`|*String*|Camera ID to subscribe, i.e. `camera0`|`/`|
|`image_src`|*String*|Name of the image topic to subscribe to|`/image_raw`|

## Notes

Remember to modify the `deploy.prototxt` to use the **FULL PATH** to the VOC label prototxt file.

Open the file `$SSDCAFFEPATH/models/VGGNet/VOC0712/SSD_512x512/deploy.prototxt`

**Change line `1803` to point to the full path of the `labelmap_voc.prototxt` file.**

