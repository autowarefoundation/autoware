# How to install Caffe for SSD

1. Complete the [pre-requisites](http://caffe.berkeleyvision.org/install_apt.html) 

2. Clone SSD Caffe fork ( It's better to do it in your home dir, if you haven't. CMake files will be looking for it there)
```
% git clone https://github.com/weiliu89/caffe.git ssdcaffe
```

3. Follow the authors' instruction to complete the requisites to compile.

4. Compile Caffe
```
make && make distribute
```

6. Download pretrained models as pointed in https://github.com/weiliu89/caffe/tree/ssd#models or use your own.

If you didn't install SSD Caffe in `ssdcaffe` inside your home, modify the ssd node's CMakeFiles and point them to your caffe directories.

Once compiled,run from the terminal, or launch from RunTimeManager

```
% roslaunch cv_tracker ssd.launch 
```
Remember to modify the launch file located inside `computing/perception/detection/packages/cv_tracker/launch/ssd.launch` and point the network and pretrained models to your paths
