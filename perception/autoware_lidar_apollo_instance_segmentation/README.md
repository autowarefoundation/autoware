# autoware_lidar_apollo_instance_segmentation

![Peek 2020-04-07 00-17](https://user-images.githubusercontent.com/8327598/78574862-92507d80-7865-11ea-9a2d-56d3453bdb7a.gif)

## Purpose

This node segments 3D pointcloud data from lidar sensors into obstacles, e.g., cars, trucks, bicycles, and pedestrians
based on CNN based model and obstacle clustering method.

## Inner-workings / Algorithms

See the [original design](https://github.com/ApolloAuto/apollo/blob/r6.0.0/docs/specs/3d_obstacle_perception.md) by Apollo.

## Inputs / Outputs

### Input

| Name               | Type                      | Description                        |
| ------------------ | ------------------------- | ---------------------------------- |
| `input/pointcloud` | `sensor_msgs/PointCloud2` | Pointcloud data from lidar sensors |

### Output

| Name                        | Type                                               | Description                                       |
| --------------------------- | -------------------------------------------------- | ------------------------------------------------- |
| `output/labeled_clusters`   | `tier4_perception_msgs/DetectedObjectsWithFeature` | Detected objects with labeled pointcloud cluster. |
| `debug/instance_pointcloud` | `sensor_msgs/PointCloud2`                          | Segmented pointcloud for visualization.           |

## Parameters

### Node Parameters

None

### Core Parameters

| Name                    | Type   | Default Value        | Description                                                                        |
| ----------------------- | ------ | -------------------- | ---------------------------------------------------------------------------------- |
| `score_threshold`       | double | 0.8                  | If the score of a detected object is lower than this value, the object is ignored. |
| `range`                 | int    | 60                   | Half of the length of feature map sides. [m]                                       |
| `width`                 | int    | 640                  | The grid width of feature map.                                                     |
| `height`                | int    | 640                  | The grid height of feature map.                                                    |
| `engine_file`           | string | "vls-128.engine"     | The name of TensorRT engine file for CNN model.                                    |
| `prototxt_file`         | string | "vls-128.prototxt"   | The name of prototxt file for CNN model.                                           |
| `caffemodel_file`       | string | "vls-128.caffemodel" | The name of caffemodel file for CNN model.                                         |
| `use_intensity_feature` | bool   | true                 | The flag to use intensity feature of pointcloud.                                   |
| `use_constant_feature`  | bool   | false                | The flag to use direction and distance feature of pointcloud.                      |
| `target_frame`          | string | "base_link"          | Pointcloud data is transformed into this frame.                                    |
| `z_offset`              | int    | 2                    | z offset from target frame. [m]                                                    |
| `build_only`            | bool   | `false`              | shutdown the node after TensorRT engine file is built                              |

## Assumptions / Known limits

There is no training code for CNN model.

### Note

This package makes use of three external codes.
The trained files are provided by apollo. The trained files are automatically downloaded when you build.

Original URL

- VLP-16 :
  <https://github.com/ApolloAuto/apollo/raw/88bfa5a1acbd20092963d6057f3a922f3939a183/modules/perception/production/data/perception/lidar/models/cnnseg/velodyne16/deploy.caffemodel>
- HDL-64 :
  <https://github.com/ApolloAuto/apollo/raw/88bfa5a1acbd20092963d6057f3a922f3939a183/modules/perception/production/data/perception/lidar/models/cnnseg/velodyne64/deploy.caffemodel>
- VLS-128 :
  <https://github.com/ApolloAuto/apollo/raw/91844c80ee4bd0cc838b4de4c625852363c258b5/modules/perception/production/data/perception/lidar/models/cnnseg/velodyne128/deploy.caffemodel>

Supported lidars are velodyne 16, 64 and 128, but you can also use velodyne 32 and other lidars with good accuracy.

1. [apollo 3D Obstacle Perception description](https://github.com/ApolloAuto/apollo/blob/r7.0.0/docs/specs/3d_obstacle_perception.md)

   ```txt
   /******************************************************************************
   * Copyright 2017 The Apollo Authors. All Rights Reserved.
   *
   * Licensed under the Apache License, Version 2.0 (the "License");
   * you may not use this file except in compliance with the License.
   * You may obtain a copy of the License at
   *
   * http://www.apache.org/licenses/LICENSE-2.0
   *
   * Unless required by applicable law or agreed to in writing, software
   * distributed under the License is distributed on an "AS IS" BASIS,
   * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
   * See the License for the specific language governing permissions and
   * limitations under the License.
   *****************************************************************************/
   ```

2. [tensorRTWrapper](https://github.com/lewes6369/tensorRTWrapper) :
   It is used under the lib directory.

   ```txt
   MIT License

   Copyright (c) 2018 lewes6369

   Permission is hereby granted, free of charge, to any person obtaining a copy
   of this software and associated documentation files (the "Software"), to deal
   in the Software without restriction, including without limitation the rights
   to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
   copies of the Software, and to permit persons to whom the Software is
   furnished to do so, subject to the following conditions:

   The above copyright notice and this permission notice shall be included in all
   copies or substantial portions of the Software.

   THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
   IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
   FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
   AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
   LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
   OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
   SOFTWARE.
   ```

3. [autoware_perception description](https://github.com/k0suke-murakami/autoware_perception/tree/feature/integration_baidu_seg/lidar_apollo_cnn_seg_detect)

   ```txt
   /*
   * Copyright 2018-2019 Autoware Foundation. All rights reserved.
   *
   * Licensed under the Apache License, Version 2.0 (the "License");
   * you may not use this file except in compliance with the License.
   * You may obtain a copy of the License at
   *
   *     http://www.apache.org/licenses/LICENSE-2.0
   *
   * Unless required by applicable law or agreed to in writing, software
   * distributed under the License is distributed on an "AS IS" BASIS,
   * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
   * See the License for the specific language governing permissions and
   * limitations under the License.
   */
   ```

### Special thanks

- [Apollo Project](https://github.com/ApolloAuto/apollo)
- [lewes6369](https://github.com/lewes6369)
- [Autoware Foundation](https://github.com/autowarefoundation/autoware)
- [Kosuke Takeuchi](https://github.com/kosuke55) (TIER IV)
