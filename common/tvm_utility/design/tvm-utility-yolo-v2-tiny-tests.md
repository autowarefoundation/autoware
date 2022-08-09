# YOLOv2 Tiny Example Pipeline {#tvm-utility-yolo-v2-tiny-tests}

This is an example implementation of an inference pipeline using the pipeline
framework. This example pipeline executes the
[YOLO V2 Tiny](https://pjreddie.com/darknet/yolov2/) model and decodes its
output.

## Compiling the Example

1. Download an example image to be used as test input. this image needs to be
   saved in the `artifacts/yolo_v2_tiny/` folder

```sh
curl https://raw.githubusercontent.com/pjreddie/darknet/master/data/dog.jpg \
  > artifacts/yolo_v2_tiny/test_image_0.jpg
```

1. Build and test with the `DOWNLOAD_ARTIFACTS` flag set.

```sh
colcon build --packages-up-to tvm_utility --cmake-args -DDOWNLOAD_ARTIFACTS=ON
colcon test --packages-select tvm_utility
```

## GPU backend

Vulkan is supported by default by the tvm_vendor package.
It can be selected by setting the `tvm_utility_BACKEND` variable:

```sh
colcon build --packages-up-to tvm_utility --cmake-args -DDOWNLOAD_ARTIFACTS=ON -Dtvm_utility_BACKEND=vulkan
```
