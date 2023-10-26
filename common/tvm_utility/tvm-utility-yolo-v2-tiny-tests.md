# YOLOv2 Tiny Example Pipeline

This is an example implementation of an inference pipeline using the pipeline
framework. This example pipeline executes the
[YOLO V2 Tiny](https://pjreddie.com/darknet/yolov2/) model and decodes its
output.

## Compiling the Example

<!-- cspell: ignore DBUILD -->

1. Check if model was downloaded during the env preparation step by ansible and
   models files exist in the folder $HOME/autoware_data/tvm_utility/models/yolo_v2_tiny.

   If not you can download them manually, see [Manual Artifacts Downloading](https://github.com/autowarefoundation/autoware/tree/main/ansible/roles/artifacts).

2. Download an example image to be used as test input. This image needs to be
   saved in the `artifacts/yolo_v2_tiny/` folder.

   ```sh
   curl https://raw.githubusercontent.com/pjreddie/darknet/master/data/dog.jpg \
   > artifacts/yolo_v2_tiny/test_image_0.jpg
   ```

3. Build.

   ```sh
   colcon build --packages-up-to tvm_utility --cmake-args -DBUILD_EXAMPLE=ON
   ```

4. Run.

   ```sh
   ros2 launch tvm_utility yolo_v2_tiny_example.launch.xml
   ```

## Parameters

| Name              | Type   | Default Value                                                           | Description                                                                                                                                  |
| ----------------- | ------ | ----------------------------------------------------------------------- | -------------------------------------------------------------------------------------------------------------------------------------------- |
| `image_filename`  | string | `$(find-pkg-share tvm_utility)/artifacts/yolo_v2_tiny/test_image_0.jpg` | Filename of the image on which to run the inference.                                                                                         |
| `label_filename`  | string | `$(find-pkg-share tvm_utility)/artifacts/yolo_v2_tiny/labels.txt`       | Name of file containing the human readable names of the classes. One class on each line.                                                     |
| `anchor_filename` | string | `$(find-pkg-share tvm_utility)/artifacts/yolo_v2_tiny/anchors.csv`      | Name of file containing the anchor values for the network. Each line is one anchor. each anchor has 2 comma separated floating point values. |
| `data_path`       | string | `$(env HOME)/autoware_data`                                             | Packages data and artifacts directory path.                                                                                                  |

## GPU backend

Vulkan is supported by default by the tvm_vendor package.
It can be selected by setting the `tvm_utility_BACKEND` variable:

```sh
colcon build --packages-up-to tvm_utility -Dtvm_utility_BACKEND=vulkan
```
