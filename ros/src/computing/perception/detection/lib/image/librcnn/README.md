# How to install Caffe for Fast-RCNN

1. Complete the [pre-requisites](http://caffe.berkeleyvision.org/install_apt.html) 


2. Clone Fast-RCNN and its Caffe fork ( It's better to do it in your home dir. CMake files will be looking for it on user's home)
```
% git clone --recursive https://github.com/rbgirshick/fast-rcnn.git
```

3. Go into `fast-rcnn/caffe-fast-rcnn` and run
`% cp Makefile.config.example Makefile.config`

4. Open `Makefile.config` with your favorite editor.
* If you have cuDNN installed uncomment the line `# USE_CUDNN := 1`
* Make sure to uncomment ` WITH_PYTHON_LAYER := 1`

5. Compile Caffe
```
make && make distribute
```

6. Download pretrained models executing while in `fast-rcnn` folder.
```
% ./data/scripts/fetch_fast_rcnn_models.sh
```
or [http://www.cs.berkeley.edu/~rbg/fast-rcnn-data/voc12_submission.tgz](http://www.cs.berkeley.edu/~rbg/fast-rcnn-data/voc12_submission.tgz)
and put it in `fast-rcnn/data/fast_rcnn_models`.


To take advantage of cuDNN, at least CUDA 7.0 and a GPU with 3.5 compute capability is required.

If you didn't install Fast-RCNN in your home, modify the librcnn and rcnn_node CMakeFiles and point them to your caffe directories.

Once compiled, if you are running from the terminal

```
% roslaunch cv_tracker rcnn.launch 
```
Remember to modify the launch file located inside `computing/perception/detection/packages/cv_tracker/launch/rcnn.launch` and point the network and pretrained models to your path
