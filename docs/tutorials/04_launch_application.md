# 4. Launch applications

Launch the perception application implemented in [autoware.universe](https://github.com/autowarefoundation/autoware.universe.git).

## 4-1. Jetson-based ECU

**NOTE: This following steps can be performed from your x86-based ECU via ssh.**

The following sample launches image-based object detection performed on two cameras individually.


```sh
cd edge-auto-jetson
source install/setup.bash

ros2 launch edge_auto_jetson_launch edge_auto_jetson.launch.xml
```
NOTE: It may take about 15 minutes before turning the results available at the first execution, which is caused by converting the ONNX model into a TensorRT engine.
From the second launch, the results should be available immediately since conversion results are cached on the disk.


This sample exploits [tensorrt_yolox](https://github.com/autowarefoundation/autoware.universe/tree/main/perception/tensorrt_yolox)
and [bytetrack](https://github.com/autowarefoundation/autoware.universe/tree/main/perception/bytetrack) packages implemented in [autoware.universe](https://github.com/autowarefoundation/autoware.universe.git).
See the READMEs of these packages for more detail.

## 4-2. x86-based ECU

The following sample launches LiDAR-based object detection and bounding-box-level fusion (i.e. late fusion) between 2D and 3D object detection.
Launch the launch file that matches your LiDAR setup.

```sh
cd edge-auto
source install/setup.bash

ros2 launch edge_auto_launch perception_at128_sample.launch.xml
## or
ros2 launch edge_auto_launch perception_xt32_sample.launch.xml
```

This sample mainly leverages [pointcloud_preprocessor](https://github.com/autowarefoundation/autoware.universe/tree/main/sensing/pointcloud_preprocessor), [centerpoint](https://github.com/autowarefoundation/autoware.universe/tree/main/perception/lidar_centerpoint), and [image_projection_based_fusion](https://github.com/autowarefoundation/autoware.universe/tree/main/perception/image_projection_based_fusion) packages
in [autoware.universe](https://github.com/autowarefoundation/autoware.universe.git).
See the READMEs of these packages for more detail.

In addition to the perception stack, this sample also launches viewers so that users can check perception results visually.
