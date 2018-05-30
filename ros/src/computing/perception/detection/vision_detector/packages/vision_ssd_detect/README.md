# How to Install Caffe for SSD

1. Complete the [pre-requisites](http://caffe.berkeleyvision.org/install_apt.html).

2. Clone the SSD Caffe fork (it is better to do it in your home directory, as CMake files will be looking for it there).
```
% git clone https://github.com/weiliu89/caffe.git ssdcaffe
% cd ssdcaffe
% git checkout 5365d0dccacd18e65f10e840eab28eb65ce0cda7
```

3. Follow the authors' instructions to complete the pre-requisites for compilation.

4. Compile Caffe:
```
make && make distribute
```

6. Download pre-trained models as provided at https://github.com/weiliu89/caffe/tree/ssd#models, or use your own.

If you didn't install SSD Caffe in `ssdcaffe` inside your home directory, modify the SSD node's CMake files and point them to your Caffe directories.

Once compiled, run from the terminal, or launch from RunTimeManager:

```
% roslaunch cv_tracker ssd.launch
```
Remember to modify the launch file located inside `computing/perception/detection/packages/cv_tracker/launch/ssd.launch` and point the network and pre-trained models to your paths.
