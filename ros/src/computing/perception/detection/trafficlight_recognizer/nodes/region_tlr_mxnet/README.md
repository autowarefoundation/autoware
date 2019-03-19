# Setup Traffic Light recognition based on MxNet

1. Clone the MxNet `v1.0.0` branch from the Apacher MxNet project GitHub onto your Home directory:

```
$ cd
$ git clone -b v1.0.0 --recursive https://github.com/apache/incubator-mxnet.git mxnet
```

2. Install CUDA 8.0 and CUDNN v5.x for CUDA 8.0 as recommended on the GitHub project.

3. Install OpenBLAS

`$ sudo apt-get install libopenblas-dev`

3. We recommend to update line 276 from the `Makefile` to only compile the code for your GPU architecture.

`KNOWN_CUDA_ARCHS := 30 35 50 52 60 61 70`

i.e. if you own a Pascal card (1060, 1070, 1080, Titan X/Xp)

change it to `KNOWN_CUDA_ARCHS := 61`

For a complete list check https://developer.nvidia.com/cuda-gpus, or execute `deviceQuery` from the CUDA samples to find yours.

4. Compile MxNet with C++, OpenCV, OpenBLAS, CUDA and CUDNN support

`$ make -j1 USE_BLAS=openblas USE_CUDA=1 USE_CUDA_PATH=/usr/local/cuda USE_CUDNN=1 USE_CPP_PACKAGE=1`

5. Compile Autoware

6. Put your trained data in `/tmp`.  

   ```
   /tmp
   ├── mxnet-network.json
   └── mxnet-network.params
   ```  
   Specify your files name in `Autoware/ros/src/computing/perception/detection/packages/trafficlight_recognizer/launch/traffic_recognition_mxnet.launch`, if you use your own.

---

For a complete reference on the compilation follow the project's documentation:
https://mxnet.incubator.apache.org/get_started/build_from_source.html
