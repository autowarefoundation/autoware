# How to install Caffe for ENet

1. Complete the [pre-requisites](http://caffe.berkeleyvision.org/install_apt.html) 

2. Clone ENEt Caffe fork in your home directory
```
% git clone --recursive https://github.com/TimoSaemann/ENet.git
```

3. Follow the authors' instruction to complete the requisites to compile as shown in
https://github.com/TimoSaemann/ENet/tree/master/Tutorial

4. Compile ENet fork of Caffe using Make (http://caffe.berkeleyvision.org/installation.html#compilation)
*Don't use CMake to compile, manually modify Makefile.config*
```
make && make distribute
```

6. Download pretrained models as pointed in https://github.com/TimoSaemann/ENet/tree/master/Tutorial#kick-start or use your own.

If you didn't install ENet Caffe in `ENet` inside your home, modify the ENet's CMakeFiles and point them to your directories.

Once compiled, run from the terminal, or launch from RunTimeManager

```
% roslaunch enet_image_segmenter enet_image_segmenter.launch 
```
Remember to modify the launch file located inside 
`computing/perception/detection/packages/enet_image_segmenter/launch/enet_image_segmenter.launch`
and point the network, pretrained models and LUT file to your paths.
