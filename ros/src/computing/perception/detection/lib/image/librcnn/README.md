# How to Install Caffe for Fast-RCNN

1. Complete the [pre-requisites](http://caffe.berkeleyvision.org/install_apt.html).


2. Clone Fast-RCNN and its Caffe fork (it is is better to do this in your home directory, as this is where CMake files will be looking for it).
```
% git clone --recursive https://github.com/rbgirshick/fast-rcnn.git
```

3. Go into `fast-rcnn/caffe-fast-rcnn` and run
`% cp Makefile.config.example Makefile.config`.

4. Open `Makefile.config` with your favorite editor.
* If you have cuDNN installed uncomment the line `# USE_CUDNN := 1`
* Make sure to uncomment ` WITH_PYTHON_LAYER := 1`

5. Compile Caffe:
```
make && make distribute
```

6. Download the pre-trained models while in `fast-rcnn` folder.
```
% ./data/scripts/fetch_fast_rcnn_models.sh
```
or [http://www.cs.berkeley.edu/~rbg/fast-rcnn-data/voc12_submission.tgz](http://www.cs.berkeley.edu/~rbg/fast-rcnn-data/voc12_submission.tgz)
and place this file in `fast-rcnn/data/fast_rcnn_models`.


To take advantage of cuDNN, at least CUDA 7.0 and a GPU with 3.5 compute capability is required.

If you didn't install Fast-RCNN in your home directory, modify the librcnn and rcnn_node CMake files and point them to your Caffe directories.

Once compiled, if you are running from the terminal:
```
% roslaunch cv_tracker rcnn.launch
```
Remember to modify the launch file located at `computing/perception/detection/packages/cv_tracker/launch/rcnn.launch` and point the network and pre-trained models to your path.
