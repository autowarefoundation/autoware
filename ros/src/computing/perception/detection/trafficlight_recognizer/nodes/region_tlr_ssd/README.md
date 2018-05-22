# Setup region_tlr_ssd

0. Refer [this-README](../../../cv_tracker/nodes/ssd/README.md) to install Caffe for SSD. The compilation process for `region_tlr_ssd` assumes that SSD Caffe has been installed under `~/ssdcaffe/`.  
   Otherwise, specify your Caffe install path in `CMakeLists.txt` for `trafficlight_recognizer` package or the compiler will skip compilation of this node.

1. Compile Autoware

2. Download and decompress [trained data](http://db3.ertl.jp/autoware/tlr_trained_model/data.tar.bz2)(or use your own) under `Autoware/ros/src/computing/perception/detection/packages/rtrafficlight_recognizer/data`.  

   Decompression result will be like followings:
   ```
   Autoware/ros/src/computing/perception/detection/packages/trafficlight_recognizer/data
   ├── Autoware_tlr_SSD_300x300_iter_60000.caffemodel
   └── deploy.prototxt
   ```  
   Specify your caffemodel and prototxt file name in `Autoware/ros/src/computing/perception/detection/packages/trafficlight_recognizer/launch/traffic_recognition_ssd.launch`, if you use your own.

3. That's it!
