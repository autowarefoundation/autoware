# How to install Caffe for ENet

1. Complete the [pre-requisites](http://caffe.berkeleyvision.org/install_apt.html) 

2. Clone ENet 

```
cd ~
git clone --recursive https://github.com/TimoSaemann/ENet.git
cd ENet/caffe-enet
```

3. Compile ENet fork of Caffe using **Make** *(Don't use CMake to compile Caffe)*
Create Makefile.config from Makefile.config.example and setup your paths
as indicated in http://caffe.berkeleyvision.org/installation.html#compilation
```
make && make distribute
```

4. Download pretrained models as pointed in 
https://github.com/TimoSaemann/ENet/tree/master/Tutorial#kick-start or use your own.

If you didn't install ENet Caffe in `ENet` inside your home for some reason, 
modify the Autoware ENet's node CMakeFile and point the paths to match your system.

Once compiled, run from the terminal

```
% roslaunch image_segmenter image_segmenter_enet.launch 
```
Remember to modify the launch file located inside 
`computing/perception/detection/packages/image_segmenter/launch/image_segmenter_enet.launch`
and configure the network configuration file, the pretrained models and the LUT file.
