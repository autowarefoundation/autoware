# CenterPoint TensorRT

This is a 3D object detection implementation of CenterPoint supporting TensorRT inference.

The object.existence_probability is stored the value of classification confidence of DNN, not probability.

## Parameters

### Input Topics

| Name             | Type        | Description                          |
| ---------------- | ----------- | ------------------------------------ |
| input/pointcloud | PointCloud2 | Point Clouds (x, y, z and intensity) |

### Output Topics

| Name                           | Type                          | Description            |
| ------------------------------ | ----------------------------- | ---------------------- |
| output/objects                 | DynamicObjectWithFeatureArray | 3D Bounding Box        |
| debug/pointcloud_densification | PointCloud2                   | multi-frame pointcloud |

### ROS Parameters

| Name                      | Type   | Description                                                 | Default |
| ------------------------- | ------ | ----------------------------------------------------------- | ------- |
| score_threshold           | float  | detected objects with score less than threshold are ignored | `0.4`   |
| densification_base_frame  | string | the base frame id to fuse multi-frame pointcloud            | `map`   |
| densification_past_frames | int    | the number of past frames to fuse with the current frame    | `1`     |
| use_encoder_trt           | bool   | use TensorRT VoxelFeatureEncoder                            | `false` |
| use_head_trt              | bool   | use TensorRT DetectionHead                                  | `true`  |
| trt_precision             | string | TensorRT inference precision: `fp32` or `fp16`              | `fp16`  |
| encoder_onnx_path         | string | path to VoxelFeatureEncoder ONNX file                       |         |
| encoder_engine_path       | string | path to VoxelFeatureEncoder TensorRT Engine file            |         |
| encoder_pt_path           | string | path to VoxelFeatureEncoder TorchScript file                |         |
| head_onnx_path            | string | path to DetectionHead ONNX file                             |         |
| head_engine_path          | string | path to DetectionHead TensorRT Engine file                  |         |
| head_pt_path              | string | path to DetectionHead TorchScript file                      |         |

## For Developers

If you have an error like `'GOMP_4.5' not found`, replace the OpenMP library in libtorch.

```bash
sudo apt install libgomp1 -y
rm /path/to/libtorch/lib/libgomp-75eea7e8.so.1
ln -s /usr/lib/x86_64-linux-gnu/libgomp.so.1 /path/to/libtorch/lib/libgomp-75eea7e8.so.1
```

## Reference

Yin, Tianwei, Xingyi Zhou, and Philipp Krähenbühl. "Center-based 3d object detection and tracking." arXiv preprint arXiv:2006.11275 (2020).

## Reference Repositories

- <https://github.com/tianweiy/CenterPoint>
- <https://github.com/open-mmlab/OpenPCDet>
- <https://github.com/poodarchu/Det3D>
- <https://github.com/xingyizhou/CenterNet>
- <https://github.com/lzccccc/SMOKE>
- <https://github.com/yukkysaito/autoware_perception>
- <https://github.com/pytorch/pytorch>
